# MATLAB Simulation
- The quadcopter supports full diagnostic recording capability, which after a flight, can be transferred to a phone
- It is a large .csv file, and the app can also run a full video recording synced with the flight
- Once you have this .csv, you can run the mex script to view this data in matlab
- You can also alter things like PID constants, and EKF tuning constants, to see how it would have changed the flight

### How to Build
```
# Make sure you are in repo_root/matlab_sim
chmod +x build_mex.sh
./build_mex.sh
```

// TODO: Finish the rest of this readme