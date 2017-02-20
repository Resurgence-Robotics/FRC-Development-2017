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
	Relay *LightRed;
	Relay *LightBlue;
	Relay *LightGreen;



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
			Lift(6),//port may change
			Gripper(0,4),
			Funnel(2,6),
			Arm_floor(1,5),
			Arm_peg(3,7)


	{

		m_pdp =new PowerDistributionPanel();
		Renc= new Encoder(2,3, true, Encoder::EncodingType::k4X);//both encoders counting forward.
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);
		LightRed= new Relay(0);
		LightGreen= new Relay(1);
		LightBlue= new Relay(2);

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
	void GyroTurnRight(int angle)
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
		gyro.Reset();
		Wait(0.1);


		while (gyro.GetAngle()<angle) //turn right 45 degrees  Tested 2/18- works; to go 90 degrees set it to 84
		{
			SetSpeed(-speed,speed);
		}
		SetSpeed(STOP);

	}
	// functions for arm
	void GyroTurnLeft(int angle)
	{
		float speed= 0.25;

		gyro.Reset();
		Wait(0.1);

		while (gyro.GetAngle()> (-1*angle)) //turn left 45 degrees
		{
			SetSpeed(speed,-speed);
		}
		SetSpeed(STOP);

	}
	void Arm_Up()
	{
		Arm_floor.Set(DoubleSolenoid::kForward);
		Arm_peg.Set(DoubleSolenoid::kForward);
		Wait(0.05);
		Gripper.Set(DoubleSolenoid::kReverse);
	}
	void Arm_Mid()
	{
		Arm_floor.Set(DoubleSolenoid::kForward);
		Arm_peg.Set(DoubleSolenoid::kReverse);
		Wait(0.25);
		Gripper.Set(DoubleSolenoid::kForward);//open gripper
		Wait(0.05);
	}
	void Arm_Down()
	{
		Arm_floor.Set(DoubleSolenoid::kReverse);
		Arm_peg.Set(DoubleSolenoid::kReverse);
		Wait(0.05);
	}
//autonomous procedures below
	void initializeRobot()
	{

		Arm_Up();
		Gripper.Set(DoubleSolenoid::kReverse); //close gripper
		Funnel.Set(DoubleSolenoid::kReverse); // close funnel
	}
	void Peg_Left()
	{
		Funnel.Set(DoubleSolenoid::kReverse);
		Arm_Up();
		Drive(102);
		Wait(1.50);
		GyroTurnRight(47);
		Drive(34);
		Wait(1.50);
		Arm_Mid();
		Drive(-20);
		Funnel.Set(DoubleSolenoid::kForward);
		Arm_Up();
		GyroTurnLeft(47);
		Drive(300);



		//Drive(x);

	}
	void Peg_Center()// tested- working
	{
		Arm_Up();
		Drive(85);  //change 100-(10)
		Wait(1.50);
		Arm_Mid();
		Drive(-20);
		Funnel.Set(DoubleSolenoid::kForward);
		//below this not tested
		Drive(-30);
		GyroTurnRight(90);
		Drive(40);
		GyroTurnLeft(90);
		Drive(300);
	}
	void Peg_Right()
	{
		Funnel.Set(DoubleSolenoid::kReverse);
		Arm_Up();
		Drive(102);
		Wait(1.50);
		GyroTurnLeft(47);
		Drive(32);
		Wait(1.50);
		Arm_Mid();
		Drive(-20);
		Funnel.Set(DoubleSolenoid::kForward);
		Arm_Up();
		GyroTurnRight(47);
		Drive(300);

	}
	void Autonomous()
	{
		Peg_Right();
	 //change
	//GyroTurnRight(45);
	//Drive(10);


		//Arm_floor.Set(DoubleSolenoid::kForward);
		//Arm_peg.Set(DoubleSolenoid::kForward);
		//Wait(0.05);
		//Gripper.Set(DoubleSolenoid::kReverse);


	//Drive(-10);




	}
	void OperatorControl()
	{

		float timeElapsed=0;
		int direction =0;
		gyro.Reset();
		while(IsOperatorControl() && IsEnabled())
		{

		//Driver1
			//drivetrain
				float threshold= 0.08;
				float JvalY=-1*stick1.GetY();//+up
				float JvalX=stick1.GetX();//+right
				float JvalZ=stick1.GetRawAxis(3); //turing when not moving
				float Scale =0.25;
				float RightOutput= (JvalY - (Scale * (JvalX)));
				float LeftOutput = (JvalY + (Scale * (JvalX)));
				float RotateOutput= (JvalZ * Scale);
				printf("\n X:%f", JvalX);
				printf("\n Y:%f", JvalY);
				printf("\n Z:%f", JvalZ);
				printf("\n RightValue:%f", RightOutput);
				printf("\n LeftValue: %f", LeftOutput);


					//SetSpeed((-1*RotateOutput), RotateOutput);  //twist and shout


				if(JvalY > threshold || JvalY < (-1*threshold))
				{
					SetSpeed(RightOutput,LeftOutput);//keep it stright
				}
				else
				{
					//SetSpeed(STOP);
					SetSpeed((-1*RotateOutput), RotateOutput);  //twist and shout
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
						timeElapsed= timeElapsed - 0.001;
						Funnel.Set(DoubleSolenoid::kForward);
						LightRed->Set(Relay::Value::kOff);
						LightGreen->Set(Relay::Value::kForward);
						LightBlue->Set(Relay::Value::kOff);

						Wait(0.05);//extend
					}else if(Funnel_Cycle==0)
					{
						float HZ =80.0;  // Hertz= actions per second
						if(timeElapsed<1/HZ)
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

							//gyro.Reset();
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
			   Wait(0.05);
			   Gripper.Set(DoubleSolenoid::kReverse);//close gripper
			   Wait(0.05);
		   }
		   else if((Gamepad.GetRawButton(3))||(stick1.GetRawButton(6))) //gear arm middle- (one open, one closed) Right hand 6; B button
		   {
			   Arm_floor.Set(DoubleSolenoid::kReverse);
			   Arm_peg.Set(DoubleSolenoid::kForward);
			   Wait(0.05);
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
		   timeElapsed= timeElapsed + 0.001;
		  Wait(0.001);
		}
	}
};

START_ROBOT_CLASS(Robot)
