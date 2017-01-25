#include "WPILib.h"
#define leftY 1
#define rightY 3
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

	Spark left1;
	Spark left2;
	Spark right1;
	Spark right2;
	Spark ARM;
	DoubleSolenoid Gripper;

public:
	Robot() :
		//initailize these in the same order they are instatiated (listed) above
			stick(0),
			left1(0),
			left2(1),
			right1(4),
			right2(5),
			ARM(2),
			Gripper(0,1)

	{
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	void Autonomous()
	{

	}
	void OperatorControl()
	{
		float threshhold=0.15;
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
			left1.Set(j1y);
			left2.Set(j1y);
			right1.Set(j2y);
			right2.Set(j2y);

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
