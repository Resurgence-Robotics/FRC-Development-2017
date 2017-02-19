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
int Funnel_Cycle=2;//initializing variable for funnel position
bool pressed=false;

class Robot: public SampleRobot
{
	//NEED TO UPDATE ECLIPESE! 2/17
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
		//initialize these in the same order they are instantiated (listed) above
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
		Renc= new Encoder(2,3, true, Encoder::EncodingType::k4X);//both encoders counting forward.
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);
		gyro.Calibrate();
		printf("\n gyro Calibrating...");
		Wait(0.005);//allow gyro to calibrate
		printf("\n gyro:%f", gyro.GetAngle());
	}

//functions for drivetrain
	void SetSpeed(float Rspeed, float Lspeed)					// tested --working on final
	{
		Right1.Set(-Rspeed);
		Right2.Set(-Rspeed);
		Left1.Set(Lspeed);
		Left2.Set(Lspeed);
	}
	void SetSpeed(float speed)									// tested --working on final
	{
		SetSpeed(speed, speed);
	}
	void DriveFRC(float outputMagnitude, float curve)			// tested --working on final
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
		}
	void drivestraightwithencoders(float target, float speed)	// tested --working on final
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

					enc=Renc->Get();
					printf("\n enc:%i",enc);
					DriveFRC(speed, kp*-1*gyro.GetAngle());
					Wait(0.01);
					}
				}
				if (target<0)//reverse incrementing negative			//UNTESTED
				{
					while(target<enc&&IsAutonomous())
					{
					enc=Renc->Get();
					printf("\n enc:%i",enc);
					DriveFRC(-speed, kp*-gyro.GetAngle());
					Wait(0.01);
					}
				}
			DriveFRC(0.0,0.0);
		}
	void Drive(float distance)									// tested --working on final( +- 3/8 in)

		{
			float wheel_radius =2.4;
			float wheel_circumference = 2*M_PI*wheel_radius;
			int PPR = 360;
			float enc_in = PPR/wheel_circumference;
			float Target = distance*enc_in;
			Lenc->Reset();
			Renc->Reset();
			printf("\n Renc: %i", Renc->Get());
			printf("\n Target:%f",Target);
			drivestraightwithencoders(Target, 0.25);

		}
	void Turn (float angle)										//needs to be tested must be within 1 degree consistently
		{

			int approachSpeed = 0.25/3;
			float speed =0.25;
			int radius = 9.95/2;//wheelbase Radius
			float wheelRadius = 2.4;
			float wheel_circumference=2*M_PI*wheelRadius;
			int PPR = 360;
			float enc_in = PPR/wheel_circumference;
			float theta = angle*M_PI/180; //math
			int arch = M_PI*radius*theta;
			float target = arch*enc_in;
			float approach= 8*enc_in;
			float Tolerance = 1*enc_in;
			printf("\n arch: %i", arch);
			printf("\n target: %f", target);
			printf("\n Encoders Reset");  //\n is new line
			Lenc->Reset();
			Renc->Reset();
			printf("\n Renc: %i", Renc->Get());
			if (angle>0)
			{
				printf("\n Turning_Right");
				while(-1*Renc->Get()<target)//turn right
				{
					float error =target- (-1*Renc->Get());
					if (error> approach )//go
					{
						SetSpeed(-speed, speed);//turning right quickly
					}
					else if (error < approach ) //go slower
					{
						SetSpeed(-approachSpeed,approachSpeed);//move more slowly as we aproach desired target
					}
					printf("\n Renc: %i", -1*Renc->Get());//print current process value
					Wait(0.05);//give loop time to process
					if( (std::abs(target-(-1*Renc->Get())))>=Tolerance)
					{
						break;//within one inch or past target
					}
				}
				SetSpeed(STOP);//stop the robot when loop is complete
			}
			else
			{
				//yet to be written
				printf("\n Turning_Left");
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
			return;
		}
	void GyroTurnRight()
	{
		float speed= 0.25;
		// for testing the gyro value
		/*gyro.Calibrate();
		while(IsAutonomous())
		{
			printf("\n GyroValue:%f",gyro.GetAngle());
			Wait(0.001);
		}
		*/

// 90 to the right is +90
//90 to the left is -90
		gyro.Calibrate();
		while (gyro.GetAngle()<45) //turn right 45 degrees  Tested 2/18- works; to go 90 degrees set it to 84
		{
			SetSpeed(-speed,speed);
		}
		SetSpeed(STOP);

	}
	// functions for arm
	void GyroTurnLeft()
	{
		float speed= 0.25;

		gyro.Calibrate();
		while (gyro.GetAngle()>-45) //turn left 45 degrees
		{
			SetSpeed(speed,-speed);
		}
		SetSpeed(STOP);

	}
	void Arm_Up()
	{
		Arm_floor.Set(DoubleSolenoid::kReverse);
		Arm_peg.Set(DoubleSolenoid::kReverse);
		Wait(0.05);
	}
	void Arm_Mid()
	{
		Arm_floor.Set(DoubleSolenoid::kForward);
		Arm_peg.Set(DoubleSolenoid::kReverse);
		Wait(0.05);
	}
	void Arm_Down()
	{
		Arm_floor.Set(DoubleSolenoid::kForward);
		Arm_peg.Set(DoubleSolenoid::kForward);
		Wait(0.05);
	}
//autonomous procedures below
	void initializeRobot()
	{

	}
	void Peg_Left()
	{
		Drive(111);
		Turn(45);
		//Drive(x);

	}
	void Peg_Center()
	{
		Drive(111);
		Arm_Mid();
		Drive(-10);
	}
	void Peg_Right()
	{

	}
	void Autonomous()
	{

	//Drive(111);
	//GyroTurnLeft();

		Arm_floor.Set(DoubleSolenoid::kForward);
		//Arm_peg.Set(DoubleSolenoid::kForward);
		//Wait(0.05);
		//Gripper.Set(DoubleSolenoid::kReverse);


	//Drive(-10);




	}
	void OperatorControl()
	{
		float threshhold= 0.10;
		float timeElapsed=0;
		int direction =0;
		gyro.Reset();
		while(IsOperatorControl() && IsEnabled())
		{

		//Driver1
			//drivetrain
				//tank drive with threshold
//						printf("\nLeft:%f",stick2.GetY());
//						if ((stick2.GetY()> threshhold) || (stick2.GetY()< -threshhold))
//						{
//							printf("\nY:%f",stick2.GetY());
//							Left1.Set(-stick2.GetY());
//							Left2.Set(-stick2.GetY());
//						}
//						else
//						{
//							Left1.Set(0.0);
//							Left2.Set(0.0);
//						}
//						printf("\nRight:%f",stick1.GetY());
//						if ((stick1.GetY()> threshhold)|| (stick1.GetY()< -threshhold))
//						{
//							Right1.Set (stick1.GetY());
//							Right2.Set (stick1.GetY());
//						}
//						else
//						{
//							Right1.Set(0.0);
//							Right2.Set(0.0);
//						}
				float kp = 0.003;
				if(timeElapsed>=0.125)
				{
					gyro.Reset();
					printf("\n gyroReset");
					timeElapsed=0;
				}
				float JvalY=-1*stick1.GetY();//+up
				float JvalX=stick1.GetX();//+right

				//							Right2.Set(0.0);
				if(JvalY>0) //if Jval is positive
				{
					direction=-1;
				}
				else if(JvalY<0) //if Jval is negative
				{
					direction=1;
				}

				if((JvalY> threshhold) || (JvalY< -threshhold))//forward and backwards
				{
					DriveFRC(JvalY, kp*direction*gyro.GetAngle());
				}
				else if(JvalX> threshhold || (JvalX< -threshhold))//turning
				{
					SetSpeed(((JvalY-JvalX)/2),(JvalY+JvalX)/2);
				}
				else
				{
					SetSpeed(STOP);
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
		   if ((Gamepad.GetRawButton(8))||(stick2.GetRawButton(1)))  //gripper open- left trigger
		   {
			   Gripper.Set(DoubleSolenoid::kForward);
			   Wait(0.05);
		   }
		   else if((Gamepad.GetRawButton(7))||(stick1.GetRawButton(1)))   //gripper close- right trigger
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
			   Arm_peg.Set(DoubleSolenoid::kReverse);
			   Wait(0.05);
			   Gripper.Set(DoubleSolenoid::kForward);//close gripper
			   Wait(0.05);
		   }
		   else if((Gamepad.GetRawButton(3))||(stick1.GetRawButton(6))) //gear arm middle- (one open, one closed) Right hand 6; B button
		   {
			   Arm_floor.Set(DoubleSolenoid::kForward);
			   Arm_peg.Set(DoubleSolenoid::kForward);
			   Wait(0.05);
			   Gripper.Set(DoubleSolenoid::kReverse);//open gripper
			   Wait(0.05);
		   }
		   else if ((Gamepad.GetRawButton(2))||(stick1.GetRawButton(2)))  //gear arm down- (both closed) Right hand 2; A button toggle
		   {
			   Arm_floor.Set(DoubleSolenoid::kReverse);
			   Arm_peg.Set(DoubleSolenoid::kForward); //changed this
			   Wait(0.05);
		   }
		   else //close gear
		   {
			   Arm_floor.Set(DoubleSolenoid::kOff);
			   Arm_peg.Set(DoubleSolenoid::kOff);
		   }
		   timeElapsed= timeElapsed + 0.001;
		  Wait(0.001);
		}
	}
};

START_ROBOT_CLASS(Robot)
