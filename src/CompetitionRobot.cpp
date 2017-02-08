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
		//initailize these in the same order they are instatiated (listed) above
			stick(0),
			Left1(3),
			Left2(4),
			Right1(1),
			Right2(2),
			ARM(5),
			Gripper(0,1)

	{
		Renc= new Encoder(2,3, true, Encoder::EncodingType::k4X);
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);
	}
	void SetSpeed(float Rspeed, float Lspeed)
	{
		Right1.Set(Rspeed);
		Right2.Set(Rspeed);
		Left1.Set(Lspeed);
		Left2.Set(Lspeed);
	}
	void SetSpeed(float speed)
	{
		SetSpeed(-speed, speed);
	}
	void Autonomous()
	{
	//This is what I added... -- untested 2/6/17--

		double Distance=300;
		//set enc to 0
		Renc->Reset();
		while(Renc->Get() <= Distance)
			{
			SetSpeed(0.50);
			printf("\n rightEncoder:%i",Renc->Get());
			Wait(0.025);
			}
			SetSpeed(STOP);
				printf("\n STOP");

			//print f the right encoder


	//	int distance = Left1->Get();
	//	int distance = Left2->Get();
	//  int distance = Right1->Get();
	//	int distance = Right2->Get();

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
