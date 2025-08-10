//
//  GPSController.swift
//  QuadController
//
//  Created by Brad Hesse on 7/30/25.
//

import Foundation
import CoreLocation
import Combine

// MARK: - GPS Ephemeris Data Structures

/// Represents GPS ephemeris data compatible with Quectel L96-M33 (MTK chipset)
struct MTKEphemerisData: Codable {
    let timestamp: Date
    let epoData: Data  // Raw EPO binary data for MTK chipset
    let validityHours: Int  // EPO data validity period in hours
}

/// Header for GPS startup fix data sent to firmware
/// Layout (little-endian):
/// - latitude:  8 bytes (Double)
/// - longitude: 8 bytes (Double)
/// - timestamp: 8 bytes (Double, seconds since 1970)
struct GPSStartupFixHeader {
    let latitude: Double
    let longitude: Double
    let timestamp: Double

    func toData() -> Data {
        var d = Data()
        func appendLE(_ value: Double) {
            var bits = value.bitPattern.littleEndian
            withUnsafeBytes(of: &bits) { d.append(contentsOf: $0) }
        }
        appendLE(latitude)
        appendLE(longitude)
        appendLE(timestamp)
        return d
    }
}

// MARK: - GPS Controller

class GPSController: NSObject, ObservableObject {
    
    static let shared = GPSController()
    
    // MARK: - Published Properties
    @Published var currentLocation: CLLocation?
    @Published var authorizationStatus: CLAuthorizationStatus = .notDetermined
    @Published var isDownloadingEphemeris: Bool = false
    @Published var lastEphemerisUpdate: Date?
    @Published var ephemerisData: MTKEphemerisData?
    @Published var errorMessage: String?
    @Published var downloadProgress: Double = 0.0
    
    // MARK: - Private Properties
    private let locationManager = CLLocationManager()
    private var cancellables = Set<AnyCancellable>()
    private let ephemerisUpdateInterval: TimeInterval = 3600 * 6 // 6 hours for EPO data
    private var hasAttemptedDownload = false  // Track if we've already tried downloading
    private var bleController: BLEController?  // Reference to BLE controller for transmission
    private var hasTransmittedEphemeris = false  // Track if we've already sent data to current connection
    
    // MediaTek EPO server URLs
    // Note: These are the official MTK EPO servers used by many GPS manufacturers
    private let mtkEPOServers = [
        "http://epodownload.mediatek.com/EPO.DAT",
        "http://epodownload.mediatek.com/EPO.MD5",
        "http://wepodownload.mediatek.com/EPO.DAT",  // Alternate server
        "http://wepodownload.mediatek.com/EPO.MD5"   // Alternate server
    ]
    
    // Quectel's AGPS server (requires authentication in production)
    private let quectelAGPSServer = "http://agps.quectel.com:2621/getfile.aspx"
    
    // MARK: - Initialization
    private override init() {
        super.init()
        setupLocationManager()
    }
    
    // MARK: - Public Methods
    
    /// Requests location permissions and starts location updates
    func requestLocationPermission() {
        switch authorizationStatus {
        case .notDetermined:
            print("Requesting location authorization")
            locationManager.requestWhenInUseAuthorization()
            break
        case .restricted, .denied:
            errorMessage = "Location access is restricted or denied. Please enable in Settings."
            print("Failed to get EPO data for GPS: \(errorMessage ?? "Unknown error")")
            break
        case .authorizedWhenInUse, .authorizedAlways:
            startLocationUpdates()
            break
        @unknown default:
            errorMessage = "Unknown authorization status"
            print("Failed to get EPO data for GPS: \(errorMessage ?? "Unknown error")")
            break
        }
    }
    
    /// Downloads GPS ephemeris data (EPO format) for Quectel L96-M33
    /// - Parameter force: If true, downloads even if data was already downloaded this session
    func downloadEphemerisData(force: Bool = false) {
        // Check if we already have valid data and haven't forced a download
        if !force && hasAttemptedDownload && isEPODataValid() {
            print("EPO data already downloaded and still valid, skipping download")
            return
        }
        
        // Check if we're already downloading
        if isDownloadingEphemeris {
            print("EPO download already in progress")
            return
        }
        
        // Request location permission if needed (required for ephemeris data context)
        if authorizationStatus == .notDetermined {
            print("Requesting location permission for GPS ephemeris data...")
            requestLocationPermission()
        }
        
        hasAttemptedDownload = true
        isDownloadingEphemeris = true
        errorMessage = nil
        downloadProgress = 0.0
        
        Task {
            do {
                // Try primary MTK EPO server first
                let ephemeris = try await fetchMTKEphemerisData()
                
                await MainActor.run {
                    self.ephemerisData = ephemeris
                    self.lastEphemerisUpdate = Date()
                    self.isDownloadingEphemeris = false
                    self.downloadProgress = 1.0
                    
                    // Check if we should transmit immediately
                    self.onEphemerisDownloadComplete()
                }
                
                print("Successfully downloaded EPO data: \(ephemeris.epoData.count) bytes")
                
            } catch {
                await MainActor.run {
                    self.errorMessage = "Failed to download EPO data: \(error.localizedDescription)"
                    self.isDownloadingEphemeris = false
                    self.downloadProgress = 0.0
                }
            }
        }
    }
    
    /// Downloads ephemeris data from Quectel's AGPS server (requires credentials)
    func downloadQuectelAGPSData(username: String, password: String) {
        isDownloadingEphemeris = true
        errorMessage = nil
        
        Task {
            do {
                let agpsData = try await fetchQuectelAGPSData(username: username, password: password)
                
                await MainActor.run {
                    self.ephemerisData = agpsData
                    self.lastEphemerisUpdate = Date()
                    self.isDownloadingEphemeris = false
                }
                
            } catch {
                await MainActor.run {
                    self.errorMessage = "Failed to download Quectel AGPS data: \(error.localizedDescription)"
                    self.isDownloadingEphemeris = false
                }
            }
        }
    }
    
    /// Checks if ephemeris data needs updating based on age
    func shouldUpdateEphemerisData() -> Bool {
        guard let lastUpdate = lastEphemerisUpdate else { return true }
        return Date().timeIntervalSince(lastUpdate) > ephemerisUpdateInterval
    }
    
    /// Returns EPO data ready for transmission to L96-M33 via PMTK commands
    func getEPODataForL96() -> Data? {
        return ephemerisData?.epoData
    }
    
    /// Generates PMTK commands to send EPO data to L96-M33
    func generatePMTKCommands(for epoData: Data) -> [String] {
        var commands: [String] = []
        
        // PMTK command to clear existing EPO data
        commands.append("$PMTK127*36")
        
        // Split EPO data into chunks (L96 can handle up to 2KB per command)
        let chunkSize = 1024  // 1KB chunks for safety
        let chunks = stride(from: 0, to: epoData.count, by: chunkSize).map {
            Array(epoData[$0..<min($0 + chunkSize, epoData.count)])
        }
        
        // Generate PMTK commands for each chunk
        for (index, chunk) in chunks.enumerated() {
            let hexString = chunk.map { String(format: "%02X", $0) }.joined()
            let command = "$PMTK722,\(index),\(chunk.count),\(hexString)"
            let checksum = calculateNMEAChecksum(for: command)
            commands.append("\(command)*\(checksum)")
        }
        
        // Command to enable EPO
        commands.append("$PMTK313,1*2E")
        
        return commands
    }
    
    // MARK: - Private Methods
    
    private func setupLocationManager() {
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.distanceFilter = 100 // Update every 100 meters
        
        // Check initial authorization status
        authorizationStatus = locationManager.authorizationStatus
    }
    
    private func startLocationUpdates() {
        locationManager.startUpdatingLocation()
    }
    
    private func stopLocationUpdates() {
        locationManager.stopUpdatingLocation()
    }
    
    /// Fetches EPO data from MediaTek servers
    private func fetchMTKEphemerisData() async throws -> MTKEphemerisData {
        // Try to download EPO.DAT file
        var lastError: Error?
        
        for server in mtkEPOServers where server.contains("EPO.DAT") {
            do {
                guard let url = URL(string: server) else {
                    throw NSError(domain: "GPSController", code: 1, userInfo: [NSLocalizedDescriptionKey: "Invalid URL"])
                }
                
                print("Attempting to download EPO data from: \(server)")
                
                var request = URLRequest(url: url)
                request.timeoutInterval = 30
                request.cachePolicy = .reloadIgnoringLocalCacheData
                
                let (data, response) = try await URLSession.shared.data(for: request)
                
                guard let httpResponse = response as? HTTPURLResponse,
                      httpResponse.statusCode == 200 else {
                    throw NSError(domain: "GPSController", code: 2, userInfo: [NSLocalizedDescriptionKey: "Invalid response"])
                }
                
                // Verify EPO data format (should start with specific header)
                guard data.count > 72,  // Minimum EPO file size
                      isValidEPOData(data) else {
                    throw NSError(domain: "GPSController", code: 3, userInfo: [NSLocalizedDescriptionKey: "Invalid EPO data format"])
                }
                
                print("Successfully downloaded EPO data: \(data.count) bytes")
                
                return MTKEphemerisData(
                    timestamp: Date(),
                    epoData: data,
                    validityHours: 168  // EPO data is typically valid for 7 days
                )
                
            } catch {
                lastError = error
                print("Failed to download from \(server): \(error)")
                continue
            }
        }
        
        throw lastError ?? NSError(domain: "GPSController", code: 4, userInfo: [NSLocalizedDescriptionKey: "All EPO servers failed"])
    }
    
    /// Fetches AGPS data from Quectel's server (requires authentication)
    private func fetchQuectelAGPSData(username: String, password: String) async throws -> MTKEphemerisData {
        // Build URL with parameters
        var components = URLComponents(string: quectelAGPSServer)!
        components.queryItems = [
            URLQueryItem(name: "user", value: username),
            URLQueryItem(name: "pwd", value: password),
            URLQueryItem(name: "type", value: "full")  // Request full EPO data
        ]
        
        guard let url = components.url else {
            throw NSError(domain: "GPSController", code: 5, userInfo: [NSLocalizedDescriptionKey: "Invalid URL"])
        }
        
        let (data, response) = try await URLSession.shared.data(from: url)
        
        guard let httpResponse = response as? HTTPURLResponse,
              httpResponse.statusCode == 200 else {
            throw NSError(domain: "GPSController", code: 6, userInfo: [NSLocalizedDescriptionKey: "Authentication failed"])
        }
        
        return MTKEphemerisData(
            timestamp: Date(),
            epoData: data,
            validityHours: 168
        )
    }
    
    /// Validates EPO data format
    private func isValidEPOData(_ data: Data) -> Bool {
        // EPO files have a specific structure:
        // - Header (72 bytes)
        // - Satellite data blocks (72 bytes each for 32 satellites)
        // Total minimum size should be 72 + (32 * 72) = 2376 bytes
        
        guard data.count >= 2376 else { return false }
        
        // Check for EPO header signature (first 4 bytes)
        // EPO files typically start with specific magic bytes
        let headerBytes = data.prefix(4)
        // Note: Actual magic bytes may vary, this is a placeholder check
        
        return true  // Basic validation passed
    }
    
    /// Calculates NMEA checksum for PMTK commands
    private func calculateNMEAChecksum(for command: String) -> String {
        var checksum: UInt8 = 0
        
        // Skip the $ and calculate XOR of all bytes between $ and *
        let commandData = command.dropFirst()  // Remove $
        
        for byte in commandData.utf8 {
            checksum ^= byte
        }
        
        return String(format: "%02X", checksum)
    }
}

// MARK: - CLLocationManagerDelegate

extension GPSController: CLLocationManagerDelegate {
    
    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        authorizationStatus = manager.authorizationStatus
        
        switch authorizationStatus {
        case .authorizedWhenInUse, .authorizedAlways:
            startLocationUpdates()
            break
        case .restricted, .denied:
            stopLocationUpdates()
            errorMessage = "Location access denied. GPS ephemeris data requires location access."
            break
        case .notDetermined:
            break
        @unknown default:
            break
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }
        
        // Only update if the location is recent and accurate enough
        let locationAge = Date().timeIntervalSince(location.timestamp)
        if locationAge < 5.0 && location.horizontalAccuracy < 50 {
            currentLocation = location
            
            // Automatically download ephemeris data once if we haven't already
            if !hasAttemptedDownload && shouldUpdateEphemerisData() {
                print("First valid location received, downloading ephemeris data")
                downloadEphemerisData()
            }
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        errorMessage = "Location update failed: \(error.localizedDescription)"
    }
}

// MARK: - EPO Data Extensions

extension GPSController {
    
    /// Provides information about the downloaded EPO data
    func getEPOInfo() -> String? {
        guard let ephemeris = ephemerisData else { return nil }
        
        let formatter = DateFormatter()
        formatter.dateStyle = .medium
        formatter.timeStyle = .short
        
        return """
        EPO Data Information:
        - Downloaded: \(formatter.string(from: ephemeris.timestamp))
        - Size: \(ephemeris.epoData.count) bytes
        - Valid for: \(ephemeris.validityHours) hours
        - Satellites: 32 (GPS)
        """
    }
    
    /// Checks if current EPO data is still valid
    func isEPODataValid() -> Bool {
        guard let ephemeris = ephemerisData else { return false }
        
        let hoursElapsed = Date().timeIntervalSince(ephemeris.timestamp) / 3600
        return hoursElapsed < Double(ephemeris.validityHours)
    }
    
    /// Checks if ephemeris data is available (regardless of validity)
    func hasEphemerisData() -> Bool {
        return ephemerisData != nil
    }
    
    /// Manually triggers ephemeris download (bypasses automatic download prevention)
    func forceDownloadEphemerisData() {
        downloadEphemerisData(force: true)
    }
    
    // MARK: - BLE Integration
    
    /// Sets up BLE controller observation for ephemeris data transmission
    func provideBLEController(_ controller: BLEController) {
        self.bleController = controller
        
        // Observe BLE connection state changes
        controller.$connectionState
            .sink { [weak self] state in
                self?.handleBLEStateChange(state)
            }
            .store(in: &cancellables)
    }
    
    /// Handles BLE connection state changes
    private func handleBLEStateChange(_ state: BLEConnectionState) {
        switch state {
        case .ready:
            // Connection is ready, check if we should transmit ephemeris data
            print("BLE connection ready, checking for ephemeris data transmission...")
            checkAndTransmitEphemerisData()
            
        case .disconnected:
            // Reset transmission flag for next connection
            hasTransmittedEphemeris = false
            
        default:
            break
        }
    }
    
    /// Checks if ephemeris data should be transmitted and sends it
    /// Data structure sent via L2CAP:
    /// - [1 byte: BLELargeDataBlobType] (added by transmitLargeDataBlob)
    /// - [8 bytes: latitude as double]
    /// - [8 bytes: longitude as double]
    /// - [8 bytes: timestamp as double (seconds since 1970)]
    /// - [N bytes: EPO data]
    private func checkAndTransmitEphemerisData() {
        guard let bleController = bleController,
              !hasTransmittedEphemeris,
              hasEphemerisData(),
              isEPODataValid(),
              let epoData = getEPODataForL96(),
              let location = currentLocation else {
            // Don't transmit without a valid location
            if bleController != nil && !hasTransmittedEphemeris && hasEphemerisData() && isEPODataValid() {
                print("Waiting for valid location before transmitting GPS ephemeris data...")
            }
            return
        }
        
        // Prepare the data packet using the header struct followed by EPO data
        let latitude = location.coordinate.latitude
        let longitude = location.coordinate.longitude
        let timestamp: Double = Date().timeIntervalSince1970

        let header = GPSStartupFixHeader(
            latitude: latitude,
            longitude: longitude,
            timestamp: timestamp
        )
        var fullData = header.toData()
        fullData.append(epoData)
        
        print("Transmitting GPS ephemeris data to quadcopter...")
        print("Location: lat=\(latitude), lon=\(longitude)")
        print("Timestamp: \(timestamp) (\(Date()))")
        print(getEPOInfo() ?? "")
        print("Total payload: 24 bytes header + \(epoData.count) bytes EPO = \(fullData.count) bytes")
        
        hasTransmittedEphemeris = true
        
        bleController.transmitLargeDataBlob(.GPSStartupFixData, fullData) { [weak self] error in
            if let error = error {
                print("Failed to transmit GPS ephemeris data: \(error)")
                // Reset flag on failure so we can retry
                self?.hasTransmittedEphemeris = false
            } else {
                print("Successfully transmitted GPS ephemeris data to ESP32")
                // Keep the flag set - we've successfully sent it for this connection
            }
        }
    }
    
    /// Called when ephemeris download completes
    private func onEphemerisDownloadComplete() {
        // Check if we should immediately transmit
        if bleController?.connectionState == .ready {
            checkAndTransmitEphemerisData()
        }
    }
}

