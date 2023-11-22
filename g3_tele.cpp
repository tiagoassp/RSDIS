	#include "ros/ros.h"
	#include "geometry_msgs/Twist.h"
	#include <wiringPi.h>
	#include <softPwm.h>
	#include "std_msgs/Float32.h"

	#define M1 28
	#define M2 29
	#define M3 22
	#define M4 23
	#define PWMA 25
	#define PWMB 26
	#define MAXSPEED 15

	void motion_twist_callback(const geometry_msgs::Twist::ConstPtr& msg);

	typedef struct {
		float kp;
		float kd;
		float ki;
		double prevError;	
		double error;
		double proportional;
		double derivative;
		double integral;
		double out;
	} PID;

	PID pid;

	const float speed = 9.8; // 9.8
	float leftMotorSpeed;
	float rightMotorSpeed;

	void init_struct() {
		pid.kp = 0.09; //0.09
		pid.kd = 0.13; //0.13
		pid.ki = 0.001; // 0.001
		pid.prevError = 0.0;
		pid.derivative = pid.error = pid.integral = pid.out = 0.0;
	}

	void PIDControl(const geometry_msgs::Twist::ConstPtr& msg) {
		pid.error = msg->angular.z;

		if(msg->linear.x == 10.0) 
		{
				softPwmWrite(PWMB, 0); 
				softPwmWrite(PWMA, 0);
				digitalWrite(M1, LOW);
				digitalWrite(M2, LOW);
				digitalWrite(M3, LOW);
				digitalWrite(M4, LOW);
				ros::Duration(1.0).sleep();
		} 
		else 
		{
			pid.proportional = pid.error * pid.kp;
			pid.derivative = pid.kd * (pid.error - pid.prevError);
			pid.integral += pid.error * pid.ki;
			
			pid.out = pid.proportional + pid.derivative + pid.integral;
			
			leftMotorSpeed = speed - pid.out;
			rightMotorSpeed = speed + pid.out;
			
			pid.prevError = pid.error;
			
			leftMotorSpeed = (leftMotorSpeed > MAXSPEED) ? MAXSPEED : (leftMotorSpeed < 0) ? 0 : leftMotorSpeed;
			rightMotorSpeed = (rightMotorSpeed > MAXSPEED) ? MAXSPEED : (rightMotorSpeed < 0) ? 0 : rightMotorSpeed;

			motion_twist_callback(msg);
		}
		
	}

	void pinSetup() {

		wiringPiSetup();
		pinMode(M1, OUTPUT);
		pinMode(M2, OUTPUT);
		pinMode(M3, OUTPUT);
		pinMode(M4, OUTPUT);
		softPwmCreate(PWMA, 0, 100);
		softPwmCreate(PWMB, 0, 100);
		
	}

	void motion_twist_callback(const geometry_msgs::Twist::ConstPtr& msg) {

		digitalWrite(M1, LOW);
		digitalWrite(M2, HIGH);
		digitalWrite(M3, LOW);
		digitalWrite(M4, HIGH);

		softPwmWrite(PWMA, leftMotorSpeed + msg->linear.x);
		softPwmWrite(PWMB, rightMotorSpeed + msg->linear.x); 

	}

	int main(int argc, char **argv) {
		init_struct();
		pinSetup();
		ros::init(argc, argv, "g3_tele");
		ros::NodeHandle n;
		ros::Subscriber subscriber_tf = n.subscribe("cmd_vel", 1000, PIDControl);
		ros::spin();

		ros::init(argc, argv, "image_processing");

		return 0;
	}
