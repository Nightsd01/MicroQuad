//
//  Use this file to import your target's public headers that you would like to expose to Swift.
//

#import <Foundation/Foundation.h>

#define USE_NSENUM
#define CROSS_PLATFORM_ENUM(_type, _name) NS_ENUM(_type, _name)

#import "../../quadcopter_firmware/main/TelemetryEvent.h"
