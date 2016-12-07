#include "WPILib.h"
#include "math.h"

class Robot: public SampleRobot
{
    CANTalon left1;
    CANTalon left2;
    CANTalon right1;
    CANTalon right2;
    Encoder *Lenc;
    Encoder *Renc;
	public:
	    Robot() :
	    	left1(0),
			left2(1),
			right1(14),
			right2(15)
	    {
	        Lenc = new Encoder(4,5, true, Encoder::EncodingType::k4X);
	        Renc = new Encoder(2,3, true, Encoder::EncodingType::k4X);
	    }

	void drive(float distance)
	{
	   float wheel_radius=3;
	   float wheel_circumference = 2 * M_PI * wheel_radius;
	   int PPR = 1440;
	   float enc_in = PPR/wheel_circumference;
	   float Target = distance*enc_in;
	   Lenc->Reset();									//resets the encoders
	   Renc->Reset();
	   printf("\n Renc: %i", Renc->Get());
	   if (distance > 0)//what direcion are we driving
	   {
		   while(Renc->Get()<Target)//while we haven't reached target
		   {
			   // drive forward
			   left1.Set(0.5);
			   left2.Set(0.5);
			   right1.Set(-0.5);
			   right2.Set(-0.5);
			   printf("\n Renc: %i", Renc->Get()); //printing a response to the rio-log
			   Wait(0.1); //wait to allow code to execute
		   }
		   //stop
		   left1.Set(0.0);
		   left2.Set(0.0);
		   right1.Set(0.0);
		   right2.Set(0.0);

	   }
	   if(distance < 0) //what direction are we driving
	   {
		   while(Renc->Get()>Target)//while we haven't reached target
		   {
			   //drive backwards
			   left1.Set(-0.5);
			   left2.Set(-0.5);
			   right1.Set(-0.5);
			   right2.Set(-0.5);
			   printf("\n Rence: %i", Renc->Get()); // printing a response to the rio-log
			   Wait(0.1); // wait to allow code to execute
		   }
		   left1.Set(0.0); // stahp
		   left2.Set(0.0);
		   right1.Set(0.0);
		   right2.Set(0.0);

	   }
	}


	void Autonomous()
	{
		drive(5);
		//results 12/6/16 -- robot did no stop
	}
	void OperatorControl()
	{}
	void Test()
	{}
};

START_ROBOT_CLASS(Robot)
