Primitive SLAM implemented by Pham Quang Minh
---------------------------------------------

This version was made by combining the source code of the Lidar and the server on the Raspberry Pi into one program.

How to set up:
	1. Upload arduinoCodes/noPID_v1.5/noPID_v1.5.ino onto Arduino. 
	2. Run ./phamslam /dev/ttyUSB0 from PhamSlam/phamSecureV1.2/
	3. Open lidar_plot_live.plt using gnuplot from PhamSlam/phamSecureV1.2/
	4. Open lidar_better_map_plot_live.plt using gnuplot from PhamSlam/phamSecureV1.2/ to see the current full map.
	5. Run ./tls-vincent-client [IPADDRESS] [PORT-5000] on your client computer.

How to operate:
	Additional commands for this version:
	- d: observe the current surrounding (lidar_plot_live.plt).
	- a: add current map data set into the full map.
	- z: clear all maps.
	To operate this version:
	- Use one of the movement commands.
	- Use 'd' to observe the surrounding after each movement.
	- Make corrective turns to fix the angle and pop all corrective commands.
	- Use 'a' to add to the full map.
	- Repeat until the end. Press 'e' to start the backtracking sequence.