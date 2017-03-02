#include "WPILib.h"
#include "CANTalon.h"
#include "math.h"
//#include <Timer.h>


#define leftY 1
#define rightY 3
#define STOP 0.0

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
	Relay *LightRed;
	Relay *LightBlue;
	Relay *LightGreen;
	AnalogInput Mode_Pot;  //used for autonomus
	DigitalInput Auto_Sw;  //switch used for autonomus


public:
	Robot() :
		//initialize these in the same order they are instantiated (listed) above
			stick1(0),
			stick2(1),
			Gamepad(2),
			Left1(0),  //ports in the roboRIO
			Left2(1),
			Right1(2),
			Right2(3),
			Lift(6),//port may change
			Gripper(0,4),
			Funnel(2,6),
			Arm_floor(1,5),
			Arm_peg(3,7),
			Mode_Pot(0),
			Auto_Sw(9)

	{

		m_pdp =new PowerDistributionPanel();
		Renc= new Encoder(2,3, true, Encoder::EncodingType::k4X);//both encoders counting forward.
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);
		LightRed= new Relay(0);  //for the LED lights
		LightGreen= new Relay(1);
		LightBlue= new Relay(2);
		CameraServer::GetInstance()->StartAutomaticCapture();

		gyro.Calibrate();  //calibrate the gyro
		printf("\n gyro Calibrating...");
		Wait(0.005);//allow gyro to calibrate
		printf("\n gyro:%f", gyro.GetAngle());
	}
// Miscellaneous functions
	// math functions
	float Map(float x, float in_min, float in_max, float out_min, float out_max){
	        // use this function to match two value's scales proportionally
	        return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	    }
	float Limit(float num)
	{
		   if (num > 1.0)
		   {
				   return 1.0;
		   }
		   if (num < -1.0)
		   {
				   return -1.0;
		   }
		   return num;
	}
	float Ponly_RPM(int RPM, int NFeedback, float CPR, float LoopTime)//should always be positive, accumulating inputs, run in a timed loop
	{
		//initialize variables
		float MotorOutput=0;
		float kp=0.05;
		int Tolerance=5;
		float nProcessValue=(NFeedback/CPR)*(LoopTime*60);     //Instantaneous-ish RPM, arithmetic test to the right-> (30000counts/1000cpr)/(1sec*(60sec/1min)) = (30rot)*(60sec)= 1800 rpm YAY!!
		float nError=RPM-nProcessValue;

			if(nError>1)
			{
				MotorOutput=1;//handles exception for motor capacity
			}
			else if(nError<0)
			{
				nError=0;//prevents us from oscillating around 0
			}
			else
			{
				MotorOutput=kp*nError;
			}
		return MotorOutput;
	}
	int GetRpm(int NFeedback, int CPR, float LoopTime)
	{
		return ((NFeedback/CPR)*(LoopTime*60));
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
		SetSpeed(speed, speed);  //(-)Right, (+) Left
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
					while((target>enc)&&(IsAutonomous()&&IsEnabled()))
					{

					enc=Renc->Get();
					printf("\n enc:%i",enc);
					DriveFRC(speed, kp*-1*gyro.GetAngle());
					Wait(0.01);
					}
				}
				if (target<0)//reverse incrementing negative			//UNTESTED
				{
					while((target<enc)&&(IsAutonomous()&&IsEnabled()))
					{
					enc=Renc->Get();
					printf("\n enc:%i",enc);
					DriveFRC(-speed, kp*gyro.GetAngle());
					Wait(0.01);
					}
				}
			DriveFRC(0.0,0.0);
		}
	void Drive(float distance)									// tested --working on final( +- 3/8 in)
//used to drive straight with encoders (measured in inches)
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
			drivestraightwithencoders(Target, 0.35);

		}
	void Turn (float angle)				//needs to be tested must be within 1 degree consistently
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
			Lenc->Reset(); //reset the encoder
			Renc->Reset();
			printf("\n Renc: %i", Renc->Get());
			if (angle>0)
			{
				printf("\n Turning_Right");  //using approach to turn so that robot does not overshoot the target(where we are trying to go) and keep going back and forth, instead it approaches it slowly.
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
			return;
		}
	void GyroTurnRight(int angle)
	{
		float speed= 0.25;
		// for testing the gyro value
// 90 to the right is +90
//90 to the left is -90
		gyro.Reset();
		Wait(0.1);


		while ((gyro.GetAngle()<angle)&&(IsAutonomous()&&IsEnabled())) //turn right 45 degrees  Tested 2/18- works; to go 90 degrees set it to 84
		{
			SetSpeed(-speed,speed);
		}
		SetSpeed(STOP);

	}
	void GyroTurnLeft(int angle)
	{
		float speed= 0.25;

		gyro.Reset();
		Wait(0.1);

		while ((gyro.GetAngle()> (-1*angle))&&(IsAutonomous()&&IsEnabled())) //turn left 45 degrees
		{
			SetSpeed(speed,-speed);
		}
		SetSpeed(STOP);

	}
	// functions for arm
	void Arm_Up()
	{
		Arm_floor.Set(DoubleSolenoid::kForward);		//arm is up
		Arm_peg.Set(DoubleSolenoid::kForward); //both pneumatic cylinders are closed
		Wait(0.05);
		Gripper.Set(DoubleSolenoid::kReverse); //close gripper
	}
	void Arm_Mid()
	{
		Arm_floor.Set(DoubleSolenoid::kForward); //floor pneumatic cylinder is closed //arm is at middle position
		Arm_peg.Set(DoubleSolenoid::kReverse); //arm peg pneumatic cylinder is open
		Wait(0.25);
		Gripper.Set(DoubleSolenoid::kForward);//open gripper
		Wait(0.05);
	}
	void Arm_Down()
	{
		Arm_floor.Set(DoubleSolenoid::kReverse);  //arm is down
		Arm_peg.Set(DoubleSolenoid::kReverse); //both pneumatic cylinders are open
		Wait(0.05);
	}
//autonomous procedures below
	void initializeRobot()  //initialization, use this before autonomous and Operator control
	{

		Arm_Up();
		Gripper.Set(DoubleSolenoid::kReverse); //close gripper
		Funnel.Set(DoubleSolenoid::kReverse); // close funnel
	}
	void Peg_Left()  //autonomus for putting the gear on the left peg
	{
		initializeRobot();
		Drive(102);
		Wait(1.50);
		GyroTurnRight(47);
		Drive(34);
		Wait(1.50);
		Arm_Mid();  //dropping off the gear
		Drive(-34);
		Arm_Up();
		GyroTurnLeft(47);
		Drive(300);
	}
	void Peg_Center()// tested- working
	{
		initializeRobot();
		Drive(85);
		Wait(0.5);
		Arm_Mid(); //drop off gear
		//below this not tested
		Drive(-50);
		GyroTurnRight(90);
		Drive(70);
		GyroTurnLeft(90);
		Drive(300);
	}
	void Peg_Right()
	{
		initializeRobot();
		Drive(102);
		Wait(1.50);
		GyroTurnLeft(47);
		Drive(32);
		Wait(1.50);
		Arm_Mid(); //drop off gear
		Drive(-32);
		Arm_Up();
		GyroTurnRight(47);
		Drive(300);

	}
//DRVERSTATION METHODS-- code entry-points below
	void Autonomous()
	{ //for the autonomus switch- needs to be tested witht the printf
		int Auto_Sel=Map(Mode_Pot.GetVoltage(), 0, 5, 1, 12);  //0 is the input min, 5 is the input max (5Volts), 1 is the output max, 12 is the output max
		printf("SW:%i\n", Auto_Sw.Get());
		printf("Mode:%i \n", Auto_Sel);

		if (Auto_Sw.Get()==true) //if we selected number 1,2, or 3 (it is true)
		{
			if (Auto_Sel==1) //if it is position 1
			{
				LightRed->Set(Relay::Value::kOff); //turn the other lights off
				LightGreen->Set(Relay::Value::kForward); //turn green light on
				LightBlue->Set(Relay::Value::kOff); //put the lights in the same order thoughout the code!

				Peg_Left(); //use peg left
			}
			else if (Auto_Sel==2)//if it is in position 2
			{
				LightRed->Set(Relay::Value::kOff);
				LightGreen->Set(Relay::Value::kForward);  //turn green light on
				LightBlue->Set(Relay::Value::kOff);

				Peg_Center(); //use peg center
			}
			else if (Auto_Sel==3)
			{
				LightRed->Set(Relay::Value::kOff);
				LightGreen->Set(Relay::Value::kForward); //turn the green light on
				LightBlue->Set(Relay::Value::kOff);

				Peg_Right();
			}
			else  //if we did not select any of the 3
			{
				LightRed->Set(Relay::Value::kForward);// turn the red light on
				LightGreen->Set(Relay::Value::kOff);//turnt the other lights off
				LightBlue->Set(Relay::Value::kOff);// do not do anything other than turning the light on

			}
		}

		else  //if we are did not select an autonomus run
		{
			LightRed->Set(Relay::Value::kForward);  //turn the light red
			LightGreen->Set(Relay::Value::kOff);
			LightBlue->Set(Relay::Value::kOff);

		}
	}
	void OperatorControl()
	{
		bool pressed=false;
		int Funnel_Cycle=2;//initializing variable for funnel position//could probably set this to zero.
		float timeElapsed=0;
		float LoopTime=0.001;
		while(IsOperatorControl() && IsEnabled())
		{

		//Driver1
			//drivetrain
				float threshold= 0.08;
				float JvalY=-1*stick1.GetY();//+up
				float JvalX=stick1.GetX();//+right
				float JvalZ=stick1.GetRawAxis(3); //turing when not moving
				float Scale =0.25; //going at 25%power
				float RightOutput= (JvalY - (Scale * (JvalX)));
				float LeftOutput = (JvalY + (Scale * (JvalX)));
				float RotateOutput= (JvalZ * 0.45); //Zaxis //going at 45% power
				printf("\n X:%f", JvalX);
				printf("\n Y:%f", JvalY);
				printf("\n Z:%f", JvalZ);
				printf("\n RightValue:%f", RightOutput);
				printf("\n LeftValue: %f", LeftOutput);
				if(JvalY > threshold || JvalY < (-1*threshold))  //if we are moving
				{
					SetSpeed(RightOutput,LeftOutput);//keep it stright
				}
				else //if we are not moving
				{

					SetSpeed((-1*RotateOutput), RotateOutput);  //twist and shout (turn)
				}

	    //the button number is found by going to driver station, and pressing the button while controllers plugged into computer(it will light up green)
		//Driver2
			//funnel
					//toggle
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
						timeElapsed= timeElapsed - LoopTime;
						Funnel.Set(DoubleSolenoid::kForward);
						LightRed->Set(Relay::Value::kOff);
						LightGreen->Set(Relay::Value::kForward);  //when the funnel is open, turn the green light on
						LightBlue->Set(Relay::Value::kOff);

						Wait(0.05);//extend
					}else if(Funnel_Cycle==0)
					{
						float HZ =80.0;  // Hertz= cycles per second
						if(timeElapsed<1/HZ) //every time the gyro resets, change the color of the lights
						{
							LightRed->Set(Relay::Value::kOff);
							LightGreen->Set(Relay::Value::kOff);
							LightBlue->Set(Relay::Value::kForward);
						}
						else if(timeElapsed < 2/HZ && timeElapsed > 1/HZ)
						{
							LightRed->Set(Relay::Value::kForward);
							LightGreen->Set(Relay::Value::kOff);
							LightBlue->Set(Relay::Value::kOff);
						}
						else
						{
							timeElapsed=0;
						}
						Funnel.Set(DoubleSolenoid::kReverse);
						Wait(0.05);//retract
					}else if(Funnel_Cycle==2)
					{
						Funnel.Set (DoubleSolenoid::kReverse);//retract
					}
			//lift
			if(Gamepad.GetRawButton(6))  //lift intake up- right bumper
			{
				Lift.Set(1);
			}
			else if(Gamepad.GetRawButton(5)&&(Gamepad.GetRawButton(9)))   //lift outtake down- left bumper and back button
			{
				Lift.Set(-1);
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
			   Arm_peg.Set(DoubleSolenoid::kForward);
			   Gripper.Set(DoubleSolenoid::kReverse);//close gripper
			   Wait(0.05);
		   }
		   else if((Gamepad.GetRawButton(3))||(stick1.GetRawButton(6))) //gear arm middle- (one open, one closed) Right hand 6; B button
		   {
			   Arm_floor.Set(DoubleSolenoid::kReverse);
			   Arm_peg.Set(DoubleSolenoid::kForward);
			   Gripper.Set(DoubleSolenoid::kForward);//open gripper
			   Wait(0.05);
		   }
		   else if ((Gamepad.GetRawButton(2))||(stick1.GetRawButton(2)))  //gear arm down- (both closed) Right hand 2; A button toggle
		   {
			   Arm_floor.Set(DoubleSolenoid::kReverse);
			   Arm_peg.Set(DoubleSolenoid::kReverse); //changed this
			   Wait(0.05);
		   }
		   else //close gear
		   {
			   Arm_floor.Set(DoubleSolenoid::kOff);
			   Arm_peg.Set(DoubleSolenoid::kOff);
		   }
		   timeElapsed= timeElapsed + LoopTime;
		  Wait(LoopTime);
		}
	}
	void Test()
	{
	Timer TimeT;
	TimeT.Reset();
		while (IsTest() && IsEnabled() )
		{
			TimeT.Start();//actually measures time instead of simply adding the loop times

			Wait(0.05);//typical waits in a tele-op loop
			Wait(0.001);

			if(TimeT.HasPeriodPassed(10000))//after 1 second clear the timer
			{
			TimeT.Reset();
			}
		}
	}
};

START_ROBOT_CLASS(Robot)
