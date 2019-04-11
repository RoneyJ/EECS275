#include "minimal_turtlebot/turtlebot_controller.h"
#include <cmath>

static int State = 0;
static uint64_t nano = 0;
static float rot_vel = 0.0;
static float theta = 0.0;

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
	//Place your code here! you can access the left / right wheel 
	//dropped variables declared above, as well as information about
	//bumper status. 
	
	//outputs have been set to some default values. Feel free 
	//to change these constants to see how they impact the robot. 
  
	//*soundValue = 0;
  
	//here are the various sound value enumeration options
	//soundValue.ON (mail turn-in)			0
	//soundValue.OFF (mail drop-off)		1
	//soundValue.RECHARGE (announce presence)	2
	//soundValue.BUTTON (receive request)		3
	//soundValue.ERROR				4
	//soundValue.CLEANINGSTART			5
	//soundValue.CLEANINGEND 			6

	theta = atan(abs(turtlebot_inputs.linearAccelY/turtlebot_inputs.linearAccelX));			

	float min_dist = 5.0;
	float angle = 0.0;
	for(int i = 0; i < turtlebot_inputs.numPoints; i++){
		float dist = turtlebot_inputs.ranges[i];
		float index_angle = i*turtlebot_inputs.angleIncrement + min_angle;
		if(dist < min_dist){
			min_dist = dist;
			angle = index_angle;
		}
	}

	switch(State){
		case 1: //move forward until one of the sensors is tripped
			if(min_dist >= 4.0){ //no object in visible distance, move straight
				*vel = 0.5;
				*ang_vel = 0.0;
			}
			else if(min_dist <= 0.5){ //object too close to robot, enter handling procedure
				*vel = 0.0;
				*ang_vel = 0.0;
				State = 6;
				nano = turtlebot_inputs.nanoSecs;
				*soundValue = 2;
				break;
			}
			else{ //object visible but not close enough to stop robot, try to avoid by turning away
				*vel = (1/7)*(min_dist) - (1/14);
				if(angle < 0)
					*ang_vel = 0.2;
				else
					*ang_vel = -0.2;
			}
			
			if(turtlebot_inputs.battVoltage < 10){
				State = 8;
				*vel = 0.0;
				*ang_vel = 0.0;
			}
			else if (turtlebot_inputs.leftBumperPressed == 1 || turtlebot_inputs.centerBumperPressed == 1 || turtlebot_inputs.sensor0State == 1 || turtlebot_inputs.sensor1State == 1){
				//bumper pressed, obstacle to left of or in front of robot
				//proceed to state 2, set future rotational velocity to turn right pi/8 rad/sec
				State = 2;
				rot_vel = -0.3927;
				nano = turtlebot_inputs.nanoSecs;
				*soundValue = 2;
			}
			else if(turtlebot_inputs.rightBumperPressed == 1 || turtlebot_inputs.sensor2State == 1){
				//bumper pressed, obstacle to right of robot
				//proceed to state 2, set future rotational velocity to turn left pi/8 rad/sec
				State = 2;
				rot_vel = 0.3927;
				nano = turtlebot_inputs.nanoSecs;
				*soundValue = 2;
			}
			else if(turtlebot_inputs.leftWheelDropped == 1 || turtlebot_inputs.rightWheelDropped == 1){
				//wheel dropped sensor active, halt all movement
				State = 4;
			}
			else if(theta > 0.349){
				//vector for linear accel passed 20 degrees, stop motors
				State = 5;
			}
			break;

		case 2: //bumper was pressed or cliff sensor tripped, move back 0.5 meters
			*vel = -0.25; //move back 0.25 m/s
			*ang_vel = 0.0;
			if(turtlebot_inputs.nanoSecs-nano >= 2000000000){ //move back for 2 sec (0.5 m)
				State = 3;
				nano = turtlebot_inputs.nanoSecs;
			}
			break;

		case 3:	//robot backed up, turn accordingly to avoid obstacle/cliff detected by sensors
			*vel = 0.0;
			*ang_vel = rot_vel; //turn pi/8 rad/s
			if(turtlebot_inputs.nanoSecs-nano >= 4000000000){ //turn for 4 sec (pi/2 rad)
				State = 1;
			}
			break;

		case 4: //wheel droppped, stop moving until bot is back on ground
			*vel = 0.0;
			*ang_vel = 0.0;
			*soundValue = 4;
			if(turtlebot_inputs.leftWheelDropped == 0 && turtlebot_inputs.rightWheelDropped == 0){ //if either wheel is pushed back up, continue driving
				State = 1;
			}
			break;

		case 5: //stop motors until linear accel vector drops below 20 degrees
			*vel = 0.0;
			*ang_vel = 0.0;
			*soundValue = 4;
			if(theta < 0.349){ //when theta drops below 20 degrees (0.349 rad), drive again
				State = 1;
			}
			break;

		case 6: //object 0.5 meters or less away
			*vel = 0.0;
			*ang_vel = 0.0;
			if(min_dist > 0.5){ //if object is removed, go back to state 1 and drive straight
				State = 1;
			}
			else if(turtlebot_inputs.nanoSecs-nano >= 15000000000){ //object not removed, turn in place until way is clear
				if(angle < 0){ //object detected in right of robot, turn left
					rot_vel = 0.3927;
				}
				else{ //object detected on left of or in front of robot, turn right
					rot_vel = -0.3927;
				}
				State = 7;
			}
			break;
		
		case 7: //object not removed, turn until path is clear
			*vel = 0.0;
			*ang_vel = rot_vel;
			
			if(min_dist > 0.5) //when path is clear again, continue to State 1 and drive straight
				State = 1;

			break;

		case 8: //battery low, halt robot until charged
			*vel = 0.0;
			*ang_vel = rot_vel;
			
			if(turtlebot_inputs.battVoltage > 10){
				State = 1;
			}
			break;
	}
}

