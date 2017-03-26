#include "WPILib.h"
#include "CANTalon.h"
#include "math.h"

#define leftY 1
#define rightY 3

void StartExcellLogging()//hopefully will be used in VS for a reason to start data logging when we get there
	{
	printf("\n <$EO$> Lenc, Renc, Gyro, Accel, Setpoint, ProcessData, Output, Auto_Sw, Run_Mode, Method, Time");
	}
	void StopExcellLoging()//will be used in VS for a reason to stop data logging
	{
	printf("\n <$EO$> STOPLOG");
	}
	
#define STOP 0.0
#define CIM_RPM 5000
class Robot: public SampleRobot
{
	//intialize class members here
	PowerDistributionPanel *m_pdp;
	ADXRS450_Gyro gyro;
	Encoder *Renc;
	Encoder *Lenc;
//	Encoder *Tenc;
	Encoder *S1enc;
	Relay *LightRed;
	Relay *LightBlue;
	Relay *LightGreen;

	Joystick stick1; // only joystick
	Joystick stick2;
	Joystick Gamepad;
	CANTalon Left1;
	CANTalon Left2;
	CANTalon Right1;
	CANTalon Right2;
	CANTalon Lift;
	CANTalon Lift2;
	CANTalon Shooter1;
	CANTalon Shooter2;
	DoubleSolenoid Gripper;
	DoubleSolenoid Funnel;
	DoubleSolenoid Arm_floor;
	DoubleSolenoid Arm_peg;
	AnalogInput Mode_Pot;  //used for autonomous
	DigitalInput Auto_Sw;  //switch used for autonomous


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
			Lift2(7),
			Shooter1(8), //may be the wrong #
			Shooter2(9), // may be the wrong #
			Gripper(0,4),
			Funnel(2,6),
			Arm_floor(1,5),
			Arm_peg(3,7),
			Mode_Pot(0),
			Auto_Sw(9)

	{

		m_pdp =new PowerDistributionPanel();

		Renc= new Encoder(2,3, true, Encoder::EncodingType::k4X);//both encoders counting forward.
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);// if counting wrong way, set it to false
//		Tenc= new Encoder(4,5, true, Encoder::EncodingType::k1X);// if counting wrong way, set it to false
		S1enc= new Encoder(4,5, true, Encoder::EncodingType::k1X);
		LightRed= new Relay(0);  //for the LED lights
		LightGreen= new Relay(1);
		LightBlue= new Relay(2);

		//CameraServer::GetInstance()->StartAutomaticCapture("cam0",0);
		//CameraServer::GetInstance()->StartAutomaticCapture("cam1",1);

		gyro.Calibrate();  //calibrate the gyro
		printf("\n gyro Calibrating...");
		Wait(0.005);//allow gyro to calibrate
		printf("\n gyro:%f", gyro.GetAngle());

	}
	
	// Miscellaneous functions- that do not call Outputs
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
	float SetRPM( int RPM_SetPoint, int RPM_Current, int MaxRPM)//should always be positive, accumulating inputs, run in a timed loop
	{
		//initialize variables
		float MotorOutput=0;
		float kp=0.008;
		float BaseSpeed=RPM_SetPoint/MaxRPM-0.05;//just slower theoretically than the desired output ish...
		float nError=RPM_SetPoint-RPM_Current;
			if(nError<0)//HOLD BASE SPEED
			{
				nError=0;
			}
			MotorOutput=kp*nError;

			if((MotorOutput<BaseSpeed))
			{
				MotorOutput=BaseSpeed;//prevents us from oscillating around 0
			}
			else if(MotorOutput>1)
			{
				MotorOutput=BaseSpeed+(0.25*BaseSpeed);//handles exception for motor capacity
			}
		return MotorOutput;
	}
	int GetRpm(int NFeedback, int CPR, float LoopTime)
	{
		return ( (abs(NFeedback)/CPR)*(LoopTime*60) );
	}
	float Float_abs(float in)
	{

		return ( in*( (in<0)*(-1)+(in>0) ) );// -40*((1)*-1)+0=40; 40*(((0)*-1)+1)=40   ABS WORKS!!!!
	}
	void ExcellOut(float Setpoint, float ProcessData, float Output,float etc1,float etc2, int MethodInUse)// <$EO$> is a special tag so we can tell when to output a chunk of data
	{
		printf("\n <$EO$> %i, %i, %f, %f, %f, %f, %f, %f, %f, %df, %f ", Lenc->Get(), Renc->Get(), gyro.GetAngle(), 0.0, Setpoint, ProcessData, Output, etc1, etc2, MethodInUse,0.0);
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
		{ //if need to change because it is not working, use Lenc instead of right, and test it to make sure it counts forward
			gyro.Reset();
			Renc->Reset();
			Lenc->Reset();
			Renc->Reset();
			ExcellOut(target,0,0,0,0,1);
			Wait(0.25);
			float kp = 0.125;
			int enc =0;
				if(target>0)//forward incrementing positive
				{
					while((target>enc)&&(IsAutonomous()&&IsEnabled()))
					{
					float correction=kp*-1*gyro.GetAngle()+0.15;
					enc=Renc->Get();
					DriveFRC(speed, correction);

					ExcellOut(target,correction,0,0,0,1);
					Wait(0.001);
					}
				}
				if (target<0)//reverse incrementing negative			//UNTESTED
				{
					while((target<enc)&&(IsAutonomous()&&IsEnabled()))
					{
					float correction=kp*gyro.GetAngle()+0.15;
					enc=Renc->Get();
					DriveFRC(-speed, correction);

					ExcellOut(target,correction,0,0,0,1);
					Wait(0.001);
					}
				}
			DriveFRC(0.0,0.0);
		}
	void drivestraightwithencoders2(float target, float speed)	// tested --working on final
			{ //if need to change because it is not working, use Lenc instead of right, and test it to make sure it counts forward
				gyro.Reset();
				Renc->Reset();
				Lenc->Reset();
				Renc->Reset();
				ExcellOut(target,0,0,0,0,1);
				float wheel_radius =2.2;
				float wheel_circumference = 2*M_PI*wheel_radius;
				int PPR = 360;
				float enc_in = PPR/wheel_circumference;
				float IPS = 0; //inches per second
				Timer DT;
				DT.Reset();
				Wait(0.25);
				float kp = 0.125;
				int enc =0;
					if(target>0)//forward incrementing positive
					{
						DT.Start();
						while((target>enc)&&(IsAutonomous()&&IsEnabled()))
						{
							float correction=kp*-1*gyro.GetAngle()+0.15;
							enc=Renc->Get();
							DriveFRC(speed, correction);
							IPS= abs((enc_in*enc)/(DT.Get()));
							ExcellOut(target,correction,IPS,0,0,1);
							if(IPS<1&&DT.Get()>0.5)
							{
								DriveFRC(speed,correction); //we are moving, drive normal
							}
							else
							{
								SetSpeed(STOP);
								break;
							}
							Wait(0.001);
						}
							DT.Stop();
					}
					if (target<0)//reverse incrementing negative			//UNTESTED
					{
						while((target<enc)&&(IsAutonomous()&&IsEnabled()))
						{
						float correction=kp*gyro.GetAngle()+0.15;
						enc=Renc->Get();
						DriveFRC(-speed, correction);

						ExcellOut(target,correction,0,0,0,1);
						Wait(0.001);
						}
					}
				DriveFRC(0.0,0.0);
			}

	void Drive(float distance)									// tested --working on final( +- 3/8 in)
//used to drive straight with encoders (measured in inches)
		{
			float wheel_radius =2.2;
			float wheel_circumference = 2*M_PI*wheel_radius;
			int PPR = 360;
			float enc_in = PPR/wheel_circumference;
			float Target = distance*enc_in;
			drivestraightwithencoders(Target, 0.35);
		}
	void GyroTurnRight(int angle)
	{
		float speed= 0.25;  //changed from 25
		// for testing the gyro value
// 90 to the right is +90
//90 to the left is -90
		gyro.Reset();
		Wait(0.1);
		ExcellOut(angle,0,0,0,0,2);
		while ((gyro.GetAngle()<angle)&&(IsAutonomous()&&IsEnabled())) //turn right 45 degrees  Tested 2/18- works; to go 90 degrees set it to 84
		{
			ExcellOut(angle,0,0,0,0,0);
			SetSpeed(-speed,speed);
		}
		SetSpeed(STOP);

	}
	void GyroTurnLeft(int angle)

	{
		float speed= 0.25;  //changed from 25
		ExcellOut(angle,0,0,0,0,3);
		gyro.Reset();
		Wait(0.1);
		while ((gyro.GetAngle()> (-1*angle))&&(IsAutonomous()&&IsEnabled())) //turn left 45 degrees
		{
			ExcellOut(angle,0,0,0,0,0);
			SetSpeed(speed,-speed);
		}
		SetSpeed(STOP);

	}
	void DriveCoast()
	{
		Left1.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
		Left2.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
		Right1.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
		Right2.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);

	}
	void DriveBrake()
	{
		Left1.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		Left2.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		Right1.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		Right2.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	}
	void EncoderShooter()
	{
		//1024 cpr

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
	void Peg_Left()  //autonomous for putting the gear on the left peg
	{
		initializeRobot();
		Drive(79); //was 83
		Wait(1.50);
		GyroTurnRight(53);
		Drive(29);  //was 34
		Wait(0.125); //changed from 1.5
		Arm_Mid();  //dropping off the gear
		Drive(-34);
		Arm_Up();
		GyroTurnLeft(50);
		drivestraightwithencoders((10*360),1.0);//drive like a bat out of hell for 10 feet.
	}
	void Peg_Center()// tested- working
	{
		initializeRobot();
		Drive(80);
		Wait(0.125);
		Arm_Mid(); //drop off gear
		//below this not tested
		Drive(-50);
		Arm_Up();
		GyroTurnRight(90);
		Drive(70);
		GyroTurnLeft(90);
		Drive(300);
	}
	void Peg_Right()
	{
		initializeRobot();
		Drive(79);//was 83
		Wait(1.50);
		GyroTurnLeft(53);
		Drive(29);  //was 34
		Wait(0.125); //changed from 1.5
		Arm_Mid(); //drop off gear
		Drive(-32);
		Arm_Up();
		GyroTurnRight(50);
		drivestraightwithencoders((10*360),1.0);//drive like a bat out of hell for 10 feet.

	}
//DRVERSTATION METHODS-- code entry-points below
	void Autonomous()
	{ //for the autonomus switch- needs to be tested witht the printf
//		int Auto_Sel=Map(Mode_Pot.GetVoltage(), 0, 5, 1, 12);  //0 is the input min, 5 is the input max (5Volts), 1 is the output max, 12 is the output max
//		printf("SW:%i\n", Auto_Sw.Get());
//		printf("Mode:%i \n", Auto_Sel);
//
//		if (Auto_Sw.Get()==true) //if we selected number 1,2, or 3 (it is true)
//		{
			LightRed->Set(Relay::Value::kOff);
			LightGreen->Set(Relay::Value::kForward);  //turn green light on
			LightBlue->Set(Relay::Value::kOff);

			Peg_Center(); //use peg center
//		}
//
//
//		else  //if we are did not select an autonomus run
//		{
//			LightRed->Set(Relay::Value::kForward);  //turn the light red
//			LightGreen->Set(Relay::Value::kOff);
//			LightBlue->Set(Relay::Value::kOff);
//
//		}
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
				
				
				float Sensitivity =Map(stick1.GetRawAxis(2),-1.0 , 1.0, 0.125, 0.5);
				float RightOutput= (JvalY - (Scale * (JvalX*1.2))); //increase turning speed when moving
				float LeftOutput = (JvalY + (Scale * (JvalX*1.2))); //increase turning speed when moving
				float RotateOutput= (/*(JvalZ * 0.45)+*/(JvalX*0.4)); //change this number to change turning speed- 0.4
				printf("\n X:%f", JvalX);
				printf("\n Y:%f", JvalY);
				printf("\n Z:%f", JvalZ);
				printf("\n RightValue:%f", RightOutput);
				printf("\n LeftValue: %f", LeftOutput);

				if(JvalY > threshold || JvalY < (-1*threshold))  //if we are moving
				{
					if(stick1.GetRawButton(5))
					{
						SetSpeed(RightOutput*Sensitivity, LeftOutput*Sensitivity);
					}
					else
					{
					SetSpeed(RightOutput,LeftOutput);//keep it stright
					}
				}
				else //if we are not moving forward
				{

					SetSpeed((-1*RotateOutput), RotateOutput);  //twist and shout (turn)
				}

	    //the button number is found by going to driver station, and pressing the button while controllers plugged into computer(it will light up green)
		//Driver2

			//funnel
					//toggle
					if(stick2.GetRawButton(5)){ // when we press the switch for the first time,
						//Gamepad.GetRawButton(1)
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
			if(stick2.GetY()>0.2)  //lift intake up- right bumper
			{  //Gamepad.GetRawButton(6)
				Lift.Set(stick2.GetY()); //joysticks scaled
				Lift2.Set(stick2.GetY());
			}
//			else if(stick2.GetY()&&(stick2.GetRawButton(7)))   //lift outtake down- left bumper and back button
//			{   //(Gamepad.GetRawButton(5)&&(Gamepad.GetRawButton(9)))
//				Lift.Set(-1);
//				Lift2.Set(-1);
//			}
			else //stop lift
			{
				Lift.Set(0.0);
				Lift2.Set(0.0);
			}

			//shooter
			if(stick2.GetRawButton(7))
			{
				Shooter1.Set(0.75);
				Shooter2.Set(0.75);
			}
			else
			{
				Shooter1.Set(0.0);
				Shooter1.Set(0.0);
			}

		//Shared
			//gripper
		   if ((stick2.GetRawButton(3))||(stick1.GetRawButton(3)))  //gripper open- left trigger
		   { //Gamepad.GetRawButton(8)
			   Gripper.Set(DoubleSolenoid::kForward);
			   Wait(0.05);
		   }
		   else if((stick2.GetRawButton(1))||(stick1.GetRawButton(1)))   //gripper close- right trigger
		   {  //Gamepad.GetRawButton(7)
			   Gripper.Set(DoubleSolenoid::kReverse);
			   Wait(0.05);
		   }
		   else //stop powering solenoid gripper
		   {
			   Gripper.Set(DoubleSolenoid::kOff);
		   }
		//arm
		   if ((stick2.GetRawButton(4))||(stick1.GetRawButton(4)))//gear arm up- (both open) Right hand 4; Y button
		   {  //Gamepad.GetRawButton(4)
			   Arm_floor.Set(DoubleSolenoid::kForward);
			   Arm_peg.Set(DoubleSolenoid::kForward);
			   Gripper.Set(DoubleSolenoid::kReverse);//close gripper
			   Wait(0.05);
		   }
		   else if((stick2.GetRawButton(6))||(stick1.GetRawButton(6))) //gear arm middle- (one open, one closed) Right hand 6; B button
		   {  //Gamepad.GetRawButton(3)
			   Arm_floor.Set(DoubleSolenoid::kReverse);
			   Arm_peg.Set(DoubleSolenoid::kForward);
			   Wait(0.08);
			   Gripper.Set(DoubleSolenoid::kForward);//open gripper
			   Wait(0.05);
		   }
		   else if ((stick2.GetRawButton(2))||(stick1.GetRawButton(2)))  //gear arm down- (both closed) Right hand 2; A button toggle
		   {  //Gamepad.GetRawButton(2)
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
	TimeT.Start();//actually measures time instead of simply adding the loop time
	int motorRPM =0;
	float Setpoint=3000;
		while (IsTest() && IsEnabled() )
		{

		//	printf("\n Tenc:%i",);


			if(TimeT.Get()>.125)//after 1/8 second clear the timer
			{
			motorRPM= GetRpm(S1enc->Get(), 41, TimeT.Get());
			S1enc->Reset();
			TimeT.Reset();
			SetSpeed((SetRPM(Setpoint, motorRPM, CIM_RPM)));
			}
			//SetSpeed(1.0);//5000 rpm
			printf("\n  RPM:%i, Output:%f, Encoder:%i", motorRPM, (SetRPM(Setpoint, motorRPM, CIM_RPM)),  S1enc->Get());





			Wait(0.005);
		}
	}
};

START_ROBOT_CLASS(Robot)
