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

	Joystick stick1; // only joystick
	Joystick stick2;
	CANTalon Left1;
	CANTalon Left2;
	CANTalon Right1;
	CANTalon Right2;



public:
	Robot() :
		//initialize these in the same order they are instantiated (listed) above
			stick1(0),
			stick2(1),
			Left1(3),  //ports in the roboRIO webpage
			Left2(15),
			Right1(2),
			Right2(20)


	{

		m_pdp =new PowerDistributionPanel();
//		Compressor *mCompressor =new Compressor(0);
		Renc= new Encoder(2,3, true, Encoder::EncodingType::k4X);//both encoders counting forward.
		Lenc= new Encoder(0,1, true, Encoder::EncodingType::k4X);// if counting wrong way, set it to false



		//CameraServer::GetInstance()->StartAutomaticCapture("cam0",0);
		//CameraServer::GetInstance()->StartAutomaticCapture("cam1",1);

		gyro.Calibrate();  //calibrate the gyro
		printf("\n gyro Calibrating...");
		Wait(0.005);//allow gyro to calibrate
		printf("\n gyro:%f", gyro.GetAngle());
		StartExcellLogging();

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

		float Float_abs(float in)
		{

			return ( in*( (in<0)*(-1)+(in>0) ) );// -40*((1)*-1)+0=40; 40*(((0)*-1)+1)=40   ABS WORKS!!!!
		}
		int GetRpm(int NFeedback, int CPR, float LoopTime)
		{
			return ( (Float_abs(NFeedback)/CPR)*(LoopTime*60) );
		}
//		void ExcellOut(float Setpoint, float ProcessData, float Output,float etc1,float etc2, int MethodInUse)// <$EO$> is a special tag so we can tell when to output a chunk of data
//		{
//			printf("\n <$EO$> %i, %i, %f, %f, %f, %f, %f, %f, %f, %f, %f ", Lenc->Get(), Renc->Get(), gyro.GetAngle(), 0.0, Setpoint, ProcessData, Output, etc1, etc2, MethodInUse, 0.0);
//		}
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
//				ExcellOut(target,0,0,0,0,1);
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

//						ExcellOut(target,correction,0,0,0,1);
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

//						ExcellOut(target,correction,0,0,0,1);
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
			float speed= 0.35; //changed from 25% to 100%
			// for testing the gyro value
	// 90 to the right is +90
	//90 to the left is -90
			gyro.Reset();
			Wait(0.1);
//			ExcellOut(angle,0,0,0,0,2);
			while ((gyro.GetAngle()<angle)&&(IsAutonomous()&&IsEnabled())) //turn right 45 degrees  Tested 2/18- works; to go 90 degrees set it to 84
			{
//				ExcellOut(angle,0,0,0,0,0);
				SetSpeed(-speed,speed);
			}
			SetSpeed(STOP);

		}
		void GyroTurnLeft(int angle)

		{
			float speed= 0.25;
//			ExcellOut(angle,0,0,0,0,3);
			gyro.Reset();
			Wait(0.1);
			while ((gyro.GetAngle()> (-1*angle))&&(IsAutonomous()&&IsEnabled())) //turn left 45 degrees
			{
//				ExcellOut(angle,0,0,0,0,0);
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
	
	//autonomous procedures below

		void Peg_Left()  //autonomus for putting the gear on the left peg
		{
			Drive(83);
			Wait(1.50);
			GyroTurnRight(50);
			Drive(34);
			Wait(1.50);
			Drive(-34);
			GyroTurnLeft(50);
			drivestraightwithencoders((10*360),1.0);//drive like a bat out of hell for 10 feet.
		}
		void Peg_Center()// tested- working
		{
			Drive(80);
			Wait(0.125);
			Drive(-50);
			GyroTurnRight(90);
			Drive(70);
			GyroTurnLeft(90);
			Drive(300);
		}
		void Peg_Right()
		{
			Drive(83);//-7
			Wait(1.50);
			GyroTurnLeft(50);
			Drive(32);
			Wait(1.50);
			Drive(-32);
			GyroTurnRight(50);
			drivestraightwithencoders((10*360),1.0);//drive like a bat out of hell for 10 feet.

		}
	void Autonomous()
	{ 
//		Peg_Left();
//		Peg_Center();
//		Peg_Right();


	}
	void OperatorControl()
	{

		float timeElapsed=0;
		float LoopTime=0.007;
		while(IsOperatorControl() && IsEnabled())
		{

		//Driver1
			//drivetrain
				float threshold= 0.08;
				float JvalY=stick1.GetY();//+up
				float JvalX=stick1.GetX();//+right
				float JvalZ=stick1.GetRawAxis(2); //turing when not moving
				float Scale =0.25; //going at 25%power
				float RightOutput= (JvalY - (Scale * (JvalX)));
				float LeftOutput = (JvalY + (Scale * (JvalX)));
				float RotateOutput= ((JvalZ * 0.45)+(JvalX*Scale)); //Zaxis //going at 45% power
//				printf("\n X:%f", JvalX);
//				printf("\n Y:%f", JvalY);
//				printf("\n Z:%f", JvalZ);
//				printf("\n RightValue:%f", RightOutput);
//				printf("\n LeftValue: %f", LeftOutput);
				if(JvalY > threshold || JvalY < (-1*threshold))  //if we are moving
				{
					SetSpeed(RightOutput,LeftOutput);//keep it stright
				}
				else //if we are not moving
				{

					SetSpeed((-1*RotateOutput), RotateOutput);  //twist and shout (turn)
				}

		   timeElapsed= timeElapsed + LoopTime;
		  Wait(LoopTime);
		}
	}
	void Test()
	{

	}
};

START_ROBOT_CLASS(Robot)
