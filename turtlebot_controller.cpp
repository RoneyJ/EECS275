#include "minimal_turtlebot/turtlebot_controller.h"
#include <cmath>
#include <algorithm>

static int State = 1;
static uint64_t nano = 0;
static float rot_vel = 0.0;
static float theta = 0.0;

float goal_x = 5;
float goal_y = 2;
float current_yaw = 0.0;
float des_yaw = 0.0;
float heading_err = 0.0;

float k_phi = 10.0;

float max_ang_vel = 0.5;

static bool start = true;
static float start_x = 0.0;
static float 

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
	
	

	/*float min_dist = 5.0;
	float angle = 0.0;
	for(int i = 0; i < turtlebot_inputs.numPoints; i++){
		float dist = turtlebot_inputs.ranges[i];
		float index_angle = i*turtlebot_inputs.angleIncrement + turtlebot_inputs.minAngle;;
		if(dist < min_dist){
			min_dist = dist;
			angle = index_angle;
		}
	}

	ROS_INFO("min_dist = %f at angle %f", min_dist, angle);*/

	
	//calculate heading error
	current_yaw = 2.0 * atan2(turtlebot_inputs.z_angle, turtlebot_inputs.orientation_omega);
	ROS_INFO("current_yaw = %f", current_yaw);
	des_yaw = atan((goal_y - turtlebot_inputs.y)/(goal_x - turtlebot_inputs.x));
	ROS_INFO("des_yaw = %f", des_yaw);
	heading_err = des_yaw - current_yaw;
	ROS_INFO("heading_err = %f", heading_err);
	switch(State){
		case 1: //move forward until one of the sensors is tripped
			/*if((min_dist < 4.0) && (min_dist > 0.5)){ //no object in visible distance, move straight
				*vel = (float)(3.0/70.0)*(min_dist) + (11.0/140.0);
				if(angle < 0)
					*ang_vel = 0.1;
				else
					*ang_vel = -0.1;
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
				*vel = 0.25;
				*ang_vel = 0.0;controller_omega = MAX_OMEGA*sat(controller_omega/MAX_OMEGA);
			}*/
			if(isnan(current_yaw) || isnan(des_yaw) || isnan(heading_err)){
				*vel = 0.0;
				*ang_vel = 0.0;
			}
			else{
				*vel = 0.25;
				float des_vel = k_phi*heading_err;
				if(des_vel > 0.5)
					*ang_vel = 0.5;
				else if(des_vel < -0.5)
					*ang_vel = -0.5;
				else
					*ang_vel = des_vel;
			
				/*if(turtlebot_inputs.battVoltage < 10){
					State = 8;
					*vel = 0.0;
					*ang_vel = 0.0;
				}

				else*/ if (turtlebot_inputs.leftBumperPressed == 1 || turtlebot_inputs.centerBumperPressed == 1 || turtlebot_inputs.sensor0State == 1 || turtlebot_inputs.sensor1State == 1){
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

		/*case 6: //object 0.5 meters or less away
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
				nano = turtlebot_inputs.nanoSecs;
				State = 7;
			}
			break;*/
		
		case 7: //object not removed, turn until path is clear
			*vel = 0.0;
			*ang_vel = rot_vel;
			
			if(turtlebot_inputs.nanoSecs-nano >= 2000000000)
			//if(min_dist > 0.5) //when path is clear again, continue to State 1 and drive straight
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

