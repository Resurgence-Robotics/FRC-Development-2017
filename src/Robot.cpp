#include "WPILib.h"
#include "math.h"
#define leftY 1
#define rightY 3
#define STOP 0.0
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

	Joystick stick; // only joystick
	Spark Left1;
	Spark Left2;
	Spark Right1;
	Spark Right2;
	Spark ARM;
	DoubleSolenoid Gripper;
	Encoder *Renc;
	Encoder *Lenc;
	ADXRS450_Gyro gyro;


public:
	Robot() :
		//initailize these in the same order they are instatiated (listed) above
			stick(0),
			Left1(0),
			Left2(1),
			Right1(4),
			Right2(5),
			ARM(2),
			Gripper(0,1)

	{
		Renc= new Encoder(0,1, true, Encoder::EncodingType::k4X);
		Lenc= new Encoder(2,3, true, Encoder::EncodingType::k4X);
	}
	void SetSpeed(float Rspeed, float Lspeed)
	{
		Right1.Set(-Rspeed);
		Right2.Set(-Rspeed);
		Left1.Set(Lspeed);
		Left2.Set(Lspeed);
	}
	void SetSpeed(float speed)
	{
		SetSpeed(-speed, speed);
	}
	void Drive(float Distance)//in inches
	{
		float D=4;
		float C=3.1416*D;
		int cpr=1000;
		int Counts =int(cpr/C*Distance);//
		float speed =.75;
		while(Renc->Get()<Counts)
		{
			SetSpeed(speed);
		}SetSpeed(STOP);
	}
	void Turn(int angle)//degrees
	{
	/*	float Kp =0.03;
			 gyro.Reset();


			 float targetHeading = gyro.GetAngle() + 45.0;
			 while (gyro.GetAngle() < targetHeading)
			 {
			    //SetSpeed(-0.25, 0.25);
			 printf("\n Turning");

			 }
			// SetSpeed(STOP);
			 printf("\n STOP");
		 */
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	void Autonomous()
	{
		Drive(12);


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


		}
	}
	void Test()
	{

	}
};

START_ROBOT_CLASS(Robot)
