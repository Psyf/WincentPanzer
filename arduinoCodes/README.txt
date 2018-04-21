NOTE TO VISITORS : Git was not used during the development process. Hence we list all our versions, for your reference.
The code has placeholders, to be replaced by baremetal code later. 
Feel free to clone and contribute to the project as you wish.  

vincent_v1.1 : primitive version
vincent_v1.2 : Introduced BackTracking phase
	//Used a stack to backtrack. 
	//On hindsight, should have used a vector/array instead 
PID_vincent_v1.3 : Explored PID library for motor control, to keep vincent moving in a straight line
	//Later revealed that our PID limit was not set. That's why it was overshooting. 
	//PID frequency was also too low. 
	//Abandoned due to lack of time. 
pidMagv_1.3.1 : Explored using a magnetometer to keep vincent moving straight, and for turns.   
	//Magnetometer did not give reliable reading. 
	//Abandoned.
noPID_v1.4 : Used harcoded corrective logic to keep vincent moving straight. 
	//Uses "unusual logic" to keep vincent moving straight. 
	//Works exceptionally well. If you can explain why correctMotors() works, please send us an email. 
noPID_v1.5 : Our FINAL version, used for assessment. Contains code for IR sensors as safety measure and hardcodes powerBraking and Turning. 
	//powerBraking implemented to prevent overshoot of distance/angle. 
	//Required a weight-balanced Vincent 
noPID_v1.6 : Explored corrective turning, WIP.  
	// Abandoned due to lack of time. 

