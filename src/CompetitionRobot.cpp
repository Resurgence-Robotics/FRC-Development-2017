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
	Joystick stick; // only joystick
	ADXRS450_Gyro gyro;
//	Joystick stick2;
//	Joystick stick3;
	CANTalon Left1;
	CANTalon Left2;
	CANTalon Right1;
	CANTalon Right2;
	CANTalon ARM;
	DoubleSolenoid Gripper;
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
			ARM(5),
			Gripper(0,1)

	{
		m_pdp =new PowerDistributionPanel();
		Renc= new Encoder(2,3, true, Encoder::EncodingType::k4X);
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
				while(Renc->Get()> -1*Target)//while we haven't reached target
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
			int radius = 13.25;//wheelbase Radius
			float wheel_circumference=2*M_PI*8;
			int PPR = 1440;
			float enc_in = PPR/wheel_circumference;
			float theta = angle*M_PI/180; //math
			int arch = M_PI*radius*theta;
			float target = -1*arch*enc_in;
			printf("\n arch: %i", arch);
			printf("/n target: %f", target);
			Wait (3.0);
			Lenc->Reset();
			Renc->Reset();
			printf("\n Renc: %i", Renc->Get());  //\n is new line
			while (Renc->Get()>target)
				{
				SetSpeed(-0.25, 0.25);//turning right
				printf("\n Renc: %i", Renc->Get());
				Wait(0.005);
				}
				SetSpeed(0.0);

			while (Renc->Get()<target)
				{
				SetSpeed(0.25, -0.25);
				printf("\n Renc: %i", Renc->Get()); //|:T | :T
				Wait(0.005);
				}
				SetSpeed(0.0);
			}
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
		Drive(20);
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
			float j1y=-1*stick.GetRawAxis(1)/2;
			float j2y=-1*stick.GetRawAxis(3)/2;
			Left1.Set(j1y);
			Left2.Set(j1y);
			Right1.Set(j2y);
			Right2.Set(j2y);
			printf("\n DT:%f", j1y);

			if(stick.GetRawButton(3))//up
			{
				ARM.Set(.75);
			}
			else if(stick.GetRawButton(2))//down
			{
				ARM.Set(-.75);
			}
			else
			{
				ARM.Set(0.0);
			}
			if(stick.GetRawButton(8))//explicitly open
			{
				Gripper.Set(DoubleSolenoid::kForward);
				Wait(0.05);
			}
			else if(stick.GetRawButton(7))//closed
			{
				Gripper.Set(DoubleSolenoid::kReverse);
				Wait(0.05);
			}
			else
			{
				Gripper.Set(DoubleSolenoid::kOff);
			}

			Wait(0.005);
		}
	}
	void Test()
	{
	}
};

START_ROBOT_CLASS(Robot)
