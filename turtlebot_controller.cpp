#include "minimal_turtlebot/turtlebot_controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>

using namespace std;

static int State = 1;
static uint64_t nano = 0;
static float rot_vel = 0.0;
static float theta = 0.0;

float goal_x;
float goal_y;
float current_yaw = 0.0;
float des_yaw = 0.0;
float heading_err = 0.0;

float k_phi = 10.0;

float dist_to_goal = 0.0;

float max_ang_vel = 0.3;

const uint64_t SEC = 1000000000l;
const float PI_8 = 0.3927;

static bool start = true;
static float start_x = 0.0;
static float start_y = 0.0;

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel)
 {
 	//calculate the current heading value of the robot
	theta = atan(abs(turtlebot_inputs.linearAccelY/turtlebot_inputs.linearAccelX));

	//calculate how far away from the goal the robot is
	dist_to_goal = sqrt(pow((goal_x - turtlebot_inputs.x),2) + pow((goal_y - turtlebot_inputs.y),2));
	if(isnan(dist_to_goal))
		dist_to_goal = 5.0;

	//at start of program or when location is lost, reset all parameters
	if(start && !(isnan(turtlebot_inputs.x)) && !(isnan(turtlebot_inputs.y))){
		//ROS_INFO("startX = %f startY = %f");
		start_x = turtlebot_inputs.x;
		start_y = turtlebot_inputs.y;
		start = false;
		//GET_GOAL(goal_x,goal_y);
		cout<<"requesting new goal pose"<<endl;
		cout<<"goal_x = "<<endl;
		cin>>goal_x;
		cout<<"goal_y = "<<endl;
		cin>>goal_y;
		ROS_INFO("Goal Pose Updated to X = %2.2f, Y = %2.2f", goal_x, goal_y);
	}			
	
	
	//parse through lidar data, find the closest object to the robot
	float min_dist = 5.0;
	float angle = 0.0;
	for(int i = 0; i < turtlebot_inputs.numPoints; i++){
		float dist = turtlebot_inputs.ranges[i];
		float index_angle = i*turtlebot_inputs.angleIncrement + turtlebot_inputs.minAngle;;
		if(dist < min_dist){
			min_dist = dist;
			angle = index_angle;
		}
	}

	
	//calculate heading error
	current_yaw = 2.0 * atan2(turtlebot_inputs.qz, turtlebot_inputs.qw);
	des_yaw = atan2((goal_y - turtlebot_inputs.y),(goal_x - turtlebot_inputs.x));
	heading_err = des_yaw - current_yaw;

	//If previous goal was completed (state = 50) or goal is nan, get new goal
	if((State == 50) || isnan(goal_x) || isnan(goal_y)){
		cout<<"requesting new goal pose"<<endl;
		cout<<"goal_x = "<<endl;
		cin>>goal_x;
		cout<<"goal_y = "<<endl;
		cin>>goal_y;

		//GET_GOAL(goal_x, goal_y);
		ROS_INFO("Goal Pose Updated to X = %2.2f, Y = %2.2f", goal_x, goal_y);
	}

	//if the robot has returned to its origin after reaching its goal, stop the robot
	if((goal_x == start_x) && (goal_y == start_y) && (State == 9)){
		State = 50;
		*vel = 0.0;
		*ang_vel = 0.0;
		*soundValue = 1;
		ROS_INFO("returned to origin");
	}
	//when the goal is updated to a new position, begin traveling
	else if((goal_x != start_x) && (goal_y != start_y) && (State == 50)){
		State = 1;
	}

	//begin control of robot to get to goal
	switch(State){
		case 1: //move forward until one of the sensors is tripped
				if((min_dist < 1.0) && (min_dist > 0.6) && (min_dist < dist_to_goal)){ //object seen but not too close, avoid by turning away
					*vel = 0.1;
					if(angle < 0)	//turn away from object
						*ang_vel = 0.25;
					else
						*ang_vel = -0.25;
				}
				else if((min_dist <= 0.6) && (min_dist < dist_to_goal)){ //object too close to robot, enter handling procedure
					*vel = 0.0;
					*ang_vel = 0.0;
					State = 6;
					nano = turtlebot_inputs.nanoSecs;
					*soundValue = 2;
					ROS_INFO("Object too close!");

					break;
				}
				else{ //no object close to robot, move normally
					//if any position reading is nan, wander around
					if(isnan(current_yaw) || isnan(des_yaw) || isnan(heading_err)){
						*vel = 0.05;
						*ang_vel = PI_8*sin(float(turtlebot_inputs.nanoSecs / SEC));
						//ROS_INFO("wandering...");
					}
					else{
						//ROS_INFO("navigating");
						if((fabs(turtlebot_inputs.x - goal_x) <= 0.25) && (fabs(turtlebot_inputs.y - goal_y) <= 0.25)){
							*vel = 0.0;
							*ang_vel = 0.0;
							State = 9;
							nano = turtlebot_inputs.nanoSecs;
							*soundValue = 0;
							ROS_INFO("goal reached");
						}
						else
							*vel = 0.1;

						float des_vel = k_phi*heading_err;
						if(des_vel > max_ang_vel)
							*ang_vel = max_ang_vel;
						else if(des_vel < -max_ang_vel)
							*ang_vel = -max_ang_vel;
						else
							*ang_vel = des_vel;
					}
				}
				
			
				/*if(turtlebot_inputs.battVoltage < 10){
					State = 8;
					*vel = 0.0;
					*ang_vel = 0.0;
				}

				else*/ if (turtlebot_inputs.leftBumperPressed == 1 || turtlebot_inputs.centerBumperPressed == 1 || turtlebot_inputs.sensor0State == 1 || turtlebot_inputs.sensor1State == 1){
					//bumper pressed, obstacle to left of or in front of robot
					//proceed to state 2, set future rotational velocity to turn right pi/8 rad/sec
					State = 2;
					rot_vel = -PI_8;
					nano = turtlebot_inputs.nanoSecs;
					*soundValue = 2;
				}
				else if(turtlebot_inputs.rightBumperPressed == 1 || turtlebot_inputs.sensor2State == 1){
					//bumper pressed, obstacle to right of robot
					//proceed to state 2, set future rotational velocity to turn left pi/8 rad/sec
					State = 2;
					rot_vel = PI_8;
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
			if(turtlebot_inputs.nanoSecs-nano >= 2*SEC){ //move back for 2 sec (0.5 m)
				State = 3;
				nano = turtlebot_inputs.nanoSecs;
			}
			break;

		case 3:	//robot backed up, turn accordingly to avoid obstacle/cliff detected by sensors
			*vel = 0.0;
			*ang_vel = rot_vel; //turn pi/8 rad/s
			if(turtlebot_inputs.nanoSecs-nano >= 4*SEC){ //turn for 4 sec (pi/2 rad)
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
			if(min_dist > 0.8){ //if object is removed, go back to state 1 and drive straight
				State = 1;
			}
			else if(turtlebot_inputs.nanoSecs-nano >= 15*SEC){ //object not removed, turn in place until way is clear
				if(angle < 0){ //object detected in right of robot, turn left
					rot_vel = PI_8;
				}
				else{ //object detected on left of or in front of robot, turn right
					rot_vel = -PI_8;
				}
				nano = turtlebot_inputs.nanoSecs;
				State = 7;
			}
			break;
		
		case 7: //object not removed, turn until path is clear
			*vel = 0.0;
			*ang_vel = rot_vel;
			
			if(turtlebot_inputs.nanoSecs-nano >= 8*SEC)
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
	
		case 9: //goal location reached, spin in place 4 times
			*vel = 0.0;
			*ang_vel = PI_8;
			if(turtlebot_inputs.nanoSecs-nano >= 16*SEC){
				goal_x = start_x;
				goal_y = start_y;
				State = 10;
			}
			break;

		case 10: //face towards origin
			*vel = 0.0;
			*ang_vel = PI_8;
			if(fabs(heading_err) <= 0.4){
				State = 1;
				ROS_INFO("goal_x = %f \t goal_y = %f", goal_x, goal_y);
			}
			break;
	}
	
}

