#include "WPILib.h"
#include "math.h"
#include "CANTalon.h"
#define leftY 1
#define rightY 3
#define STOP 0.0
long Map(float x, float in_min, float in_max, float out_min, float out_max){
	// use this function to match two value's scales proportionally
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
int Funnel_Cycle=2;
bool pressed=false;

class Robot: public SampleRobot
{
	//intialize class members here
	PowerDistributionPanel *m_pdp;
	ADXRS450_Gyro gyro;
	Joystick stick1; // only joystick
	Joystick stick2;
	Joystick Gamepad;
	CANTalon Left1;
	CANTalon Left2;
	CANTalon Right1;
	CANTalon Right2;
	CANTalon Lift;
	DoubleSolenoid Gripper;
	DoubleSolenoid Funnel;
	DoubleSolenoid Arm_floor;
	DoubleSolenoid Arm_peg;
	Encoder *Renc;
	Encoder *Lenc;



public:
	Robot() :
		//initialize these in the same order they are instatiated (listed) above
			stick1(0),
			stick2(1),
			Gamepad(2),
			Left1(0),
			Left2(1),
			Right1(2),
			Right2(3),
			Lift(5),//port may change
			Gripper(0,4),
			Funnel(2,6),
			Arm_floor(1,5),
			Arm_peg(3,7)


	{
		m_pdp =new PowerDistributionPanel();
		Renc= new Encoder(2,3, false, Encoder::EncodingType::k4X);//both encoders counting forward.
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);
	}


	void SetSpeed(float Rspeed, float Lspeed)//tested working--Practice Bot
	{
		Right1.Set(-Rspeed);
		Right2.Set(-Rspeed);
		Left1.Set(Lspeed);
		Left2.Set(Lspeed);
	}
	void SetSpeed(float speed)//tested working--Practice Bot
	{
		SetSpeed(speed, speed);
	}
	void Drive(float distance)//tested working--Practice Bot

		{
			float wheel_radius =2;
			float wheel_circumference = 2*M_PI*wheel_radius;
			int PPR = 360*4;
			float enc_in = PPR/wheel_circumference;
			float Target = distance*enc_in;
			Lenc->Reset();
			Renc->Reset();
			printf("\n Renc: %i", Renc->Get());
			printf("\n Target:%f",Target);
			if(distance > 0)//what direction are we driving
			{
				while(Renc->Get()> Target)//while we haven't reached target
				{
					// drive forward
					SetSpeed(0.5);
					printf("\n Renc: %i", Renc->Get()); // printing a response to the rio-log
					Wait(0.001);//wait to allow code time to execute
				}
				SetSpeed(STOP);
			}
			if(distance < 0) //what direction are we driving
			{
				while(Renc->Get()>Target)//while we haven't reached target
				{
					// drive backwards
					SetSpeed(-0.5);
					printf("\n Renc: %i", Renc->Get()); // printing a response to the rio-log
					Wait(0.001); // wait to allow code to execute
				}
				SetSpeed(STOP);

			}
		}
	void Turn (float angle)
			{

			int approachSpeed = 0.25/3;
			float speed =0.25;
			int radius = 13.25/2;//wheelbase Radius
			float wheelRadius = 2;
			float wheel_circumference=2*M_PI*wheelRadius;
			int PPR = 1440;
			float enc_in = PPR/wheel_circumference;
			float theta = angle*M_PI/180; //math
			int arch = M_PI*radius*theta;
			float target = arch*enc_in;
			float approach= 6.0*enc_in;
			printf("\n arch: %i", arch);
			printf("\n target: %f", target);
			Wait (0.50);
			Lenc->Reset();
			Renc->Reset();
			printf("\n Encoders->Reset()");  //\n is new line
			if (angle>0)
			{
				while(Renc->Get()<target &&IsAutonomous())//move towards target
				{
					if (Renc->Get()> approach )//go faster
					{
						SetSpeed(-speed, speed);//turning right quickly
					}
					else if (Renc->Get()< approach ) //go slower
					{
						SetSpeed(-approachSpeed,approachSpeed);//move more slowly as we aproach desired target
					}
					printf("\n Renc: %i", Renc->Get());//print current process value
					Wait(0.005);//give loop time to process
				}
				SetSpeed(STOP);//stop the robot when loop is complete
			}
//			else if (angle<0)
//			{
//				while (Renc->Get()< approach ) //go slow
//					{
//					SetSpeed (-approachSpeed,approachSpeed);//tunr left
//					}
//				while (Renc->Get()> approach )
//					{
//					SetSpeed(speed, -speed);
//					printf("\n Renc: %i", Renc->Get());
//					Wait(0.005);
//					}
//
//					SetSpeed(STOP);
//
//			}

			}
/*	while (Renc->Get()<target &&IsAutonomous())
				{
				SetSpeed(-speed, speed);//turning right
				printf("\n Renc: %i", Renc->Get());
				Wait(0.005);
				}
				SetSpeed(STOP);


			while (Renc->Get()>target &&IsAutonomous())
				{
				SetSpeed(speed, -speed);
				printf("\n Renc: %i", Renc->Get()); //|:T | :T
				Wait(0.005);
				}
				SetSpeed(STOP);
			*/
	void DriveFRC(float outputMagnitude, float curve)
		{
		float leftOutput, rightOutput;
		float m_sensitivity = 0.5;
		if (curve < 0)
		{
		   float value = log(-curve);
		   float ratio = (value - m_sensitivity)/(value + m_sensitivity);
		   if (ratio == 0) ratio =.0000000001;
		  leftOutput = outputMagnitude / ratio;
		  rightOutput = outputMagnitude;
		}
		else if (curve > 0)
		{
		  float value = log(curve);
		  float ratio = (value - m_sensitivity)/(value + m_sensitivity);
		   if (ratio == 0) ratio =.0000000001;
		   leftOutput = outputMagnitude;
		   rightOutput = outputMagnitude / ratio;
		}
		else
		{
		  leftOutput = outputMagnitude;
		  rightOutput = outputMagnitude;
		}
		SetSpeed(rightOutput, leftOutput);
//		left1.Set(-1*leftOutput);
//		left2.Set(-1*leftOutput);
//		right1.Set(rightOutput);
//		right2.Set(rightOutput);
		}
	void drivestraight(float time, float speed)
	{
		gyro.Reset();
		Wait(1.0);
		float kp = 0.003;
		float TimeElapsed =0.0;
		while(TimeElapsed<time)
		{
			DriveFRC(speed, kp*gyro.GetAngle());
			Wait(0.2);
			TimeElapsed=TimeElapsed+0.2;
		}
		DriveFRC(0.0,0.0);
	}
	void drivestraightwithencoders(float target, float speed)
	{
		gyro.Reset();
		Renc->Reset();
		Wait(1.0);
		float kp = 0.003;
		int enc =0;
			if(target>0)//forward incrementing positive
			{
				while(target>enc&&IsAutonomous())
				{

				enc=-1*Renc->Get();

				printf("\n enc:%i",enc);
				DriveFRC(speed, kp*gyro.GetAngle());
				Wait(0.01);
				}
			}
			if (target<0)//reverse incrementing negative
			{
				while(target<enc&&IsAutonomous())
				{
				enc=Renc->Get();
				printf("\n enc:%i",enc);
				DriveFRC(speed, kp*-1*gyro.GetAngle());
				Wait(0.01);
				}
			}
		DriveFRC(0.0,0.0);
	}
	void Autonomous()
	{
		//Drive(20);
		Turn(90);
	}
	void OperatorControl()
	{
		float threshhold= 0.015;
		while(IsOperatorControl() && IsEnabled())
		{
		//Driver1
			//drivetrain
				//tank drive with threshold
						printf("\nY:%f",stick2.GetY());
						if (abs(stick2.GetY())> threshhold)
						{
							printf("\nY:%f",stick2.GetY());
							Left1.Set(-1.0*stick2.GetY());
							Left2.Set(-1.0*stick2.GetY());
						}
						else
						{
							Left1.Set(0.0);
							Left2.Set(0.0);
						}
						if (abs(stick1.GetY())> threshhold)
						{
							Right1.Set (-1*stick1.GetY());
							Right2.Set (-1*stick1.GetY());
						}
						else
						{
							Right1.Set(0.0);
							Right2.Set(0.0);
						}

		//Driver2
			//funnel

					if(Gamepad.GetRawButton(1)){ // when we press the switch for the first time,
							if(!pressed) { // set as pressed
								if(Funnel_Cycle==1) { // when we press it again, it gets turned off
									Funnel_Cycle=0;
								}else{
									Funnel_Cycle= 1;
								}
							}
						pressed = true; // keeping track of pressed allows the button to be
					}else{ // held down
						pressed = false;
					}
					if(Funnel_Cycle==1)
					{
						Funnel.Set(DoubleSolenoid::kForward);
						Wait(0.05);//extend
					}else if(Funnel_Cycle==0)
					{
						Funnel.Set(DoubleSolenoid::kReverse);
						Wait(0.05);//retract
					}else if(Funnel_Cycle==2)
					{
						Funnel.Set (DoubleSolenoid::kReverse);//retract
					}
			//lift
			if(Gamepad.GetRawButton(6))  //lift intake up- right bumper
			{
				Lift.Set(0.75);
			}
			else if(Gamepad.GetRawButton(5))   //lift outtake down- left bumper
			{
				Lift.Set(-0.75);
			}
			else //stop lift
			{
				Lift.Set(0.0);
			}

		//Shared
			//gripper
		   if ((Gamepad.GetRawButton(7))||(stick2.GetRawButton(1)))  //gripper open- left trigger
		   {
			   Gripper.Set(DoubleSolenoid::kForward);
			   Wait(0.05);
		   }
		   else if((Gamepad.GetRawButton(8))||(stick1.GetRawButton(1)))   //gripper close- right trigger
		   {
			   Gripper.Set(DoubleSolenoid::kReverse);
			   Wait(0.05);
		   }
		   else //close gripper
		   {
			   Gripper.Set(DoubleSolenoid::kOff);
		   }
		//arm
		   if ((Gamepad.GetRawButton(4))||(stick1.GetRawButton(4)))//gear arm up- (both open) Right hand 4; Y button
		   {
			   Arm_floor.Set(DoubleSolenoid::kForward);
			   Arm_peg.Set(DoubleSolenoid::kForward);
			   Wait(0.05);
		   }
		   else if((Gamepad.GetRawButton(3))||(stick1.GetRawButton(6))) //gear arm middle- (one open, one closed) Right hand 6; B button
		   {
			   Arm_floor.Set(DoubleSolenoid::kForward);
			   Arm_peg.Set(DoubleSolenoid::kReverse);
			   Wait(0.05);
		   }
		   else if ((Gamepad.GetRawButton(2))||(stick1.GetRawButton(2)))  //gear arm down- (both closed) Right hand 2; A button toggle
		   {
			   Arm_floor.Set(DoubleSolenoid::kReverse);
			   Arm_peg.Set(DoubleSolenoid::kReverse);
			   Wait(0.05);
		   }
		   else //close gear
		   {
			   Arm_floor.Set(DoubleSolenoid::kOff);
			   Arm_peg.Set(DoubleSolenoid::kOff);
		   }
		  Wait(0.001);
		}
	}
};

START_ROBOT_CLASS(Robot)
