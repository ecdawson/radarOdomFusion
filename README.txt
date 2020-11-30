Initial Pose:

"x_m": 46.57,
"y_m": 30.91,
"azimuth_deg": -179.6967,


-----------------------------
Map_file:

- When plotting the map file, plot it as points not a line
- You can plot the map using the command: plot(floor_map_pcl(:,1), floor_map_pcl(:,2), '.')


-----------------------------
Ref_soln:

- The ref soln file has 4 colmns: timestamp, x (meters), y(meters), azimuth (rad) 

-----------------------------
Vehicle_info:

 - Speed data rate is 100 Hz
 - Gear data rate is 5 Hz  (0: parking, 1: reverse, 2: neutral, 3: forward)
 - Wheel pulses data rate is 50 Hz
 - The time stamp of the vehicle info (and IMU) can be obtained from "field.header.stamp" column by multiplying it by 10e-6 to mtach the radar and ref solution data
 - The wheel pulses parameters are as follows:
    distance_between_wheels = 1.556; % m
    rl_wheel_scale = 0.97861 * (740 * pi * 1e-3 / 96); % m/pulse
    rr_wheel_scale = 0.97689 * (740 * pi * 1e-3 / 96); % m/pulse
    
    fl_wheel_scale = 1.06446 * (735 * pi * 1e-3 / 96); % m/pulse
    fr_wheel_scale = 1.06514 * (735 * pi * 1e-3 / 96); % m/pulse
    max_pulses = 65536;
	
-----------------------------
Radar data:

- The radar data rate is 20 Hz
- Please use the "radar_offsets.csv" file
	
 
