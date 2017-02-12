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
/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot
{
	//intialize class members here
	PowerDistributionPanel *m_pdp;
	ADXRS450_Gyro gyro;
	Joystick stick; // only joystick
//	Joystick stick2;
//	Joystick stick3;
	CANTalon Left1;
	CANTalon Left2;
	CANTalon Right1;
	CANTalon Right2;
	CANTalon ARM;
//	CANTalon Lift;
	DoubleSolenoid Gripper;
	DoubleSolenoid Solenoid2;
	DoubleSolenoid Solenoid3;
	DoubleSolenoid Solenoid4;
	Encoder *Renc;
	Encoder *Lenc;



public:
	Robot() :
		//initialize these in the same order they are instatiated (listed) above
			stick(0),
			Left1(0),
			Left2(1),
			Right1(2),
			Right2(3),
			ARM(5),//port may change
			Gripper(0,4),//Names need to be properly assigned, ordered pair (A, B) matches solenoids 
			Solenoid2(1,5),
			Solenoid3(2,6),
			Solenoid4(3,7)


	{
		m_pdp =new PowerDistributionPanel();
		Renc= new Encoder(2,3, false, Encoder::EncodingType::k4X);//both encoders counting forward.
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);
	}

	//{
	//Solenoid1 = new Solenoid(0);
	//Solenoid1->Set(true);
	//}

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
		//float threshhold=0.15;
		while(IsOperatorControl() && IsEnabled())
		{
//			if (abs(stick.GetRawAxis(1))> threshhold)
//			{
//				left1.Set(-1*stick.GetRawAxis(1));
//				left2.Set(-1*stick.GetRawAxis(1));
//			}
//			else
//			{
//				left1.Set(0.0);
//				left2.Set(0.0);
//			}
//			if (abs(stick.GetRawAxis(rightY))> threshhold)
//			{
//				right1.Set (-1*stick.GetRawAxis(3));
//				right2.Set (-1*stick.GetRawAxis(3));
//			}
//			else
//			{
//				right1.Set(0.0);
//				right2.Set(0.0);
//			}
			Wait(0.005);
		}
	}
	void Test()//test method created to allow manual control of all pnumatics and ensure functionality
	{
		while(IsEnabled()&&IsTest())
		{
			if(stick.GetRawButton(7)&& stick.GetRawButton(1))//explicitly open
			{
				Gripper.Set(DoubleSolenoid::kForward);
				Wait(0.05);
			}
			else if(stick.GetRawButton(7)&& stick.GetRawButton(2))//closed
			{
				Gripper.Set(DoubleSolenoid::kReverse);
				Wait(0.05);
			}
			else
			{
				Gripper.Set(DoubleSolenoid::kOff);
			}
			if(stick.GetRawButton(1))//explicitly open
			{
				Solenoid2.Set(DoubleSolenoid::kForward);
				Wait(0.05);
			}
			else if(stick.GetRawButton(2))//closed
			{
				Solenoid2.Set(DoubleSolenoid::kReverse);
				Wait(0.05);
			}
			else
			{
				Solenoid2.Set(DoubleSolenoid::kOff);
			}

			if(stick.GetRawButton(3))//explicitly open
			{
				Solenoid3.Set(DoubleSolenoid::kForward);
				Wait(0.05);
			}
			else if(stick.GetRawButton(5))//closed
			{
				Solenoid3.Set(DoubleSolenoid::kReverse);
				Wait(0.05);
			}
			else
			{
				Solenoid3.Set(DoubleSolenoid::kOff);
			}

			if(stick.GetRawButton(4))//explicitly open
			{
				Solenoid4.Set(DoubleSolenoid::kForward);
				Wait(0.05);
			}
			else if(stick.GetRawButton(6))//closed
			{
				Solenoid4.Set(DoubleSolenoid::kReverse);
				Wait(0.05);
			}
			else
			{
				Solenoid4.Set(DoubleSolenoid::kOff);
			}
		}
	}
};

START_ROBOT_CLASS(Robot)
