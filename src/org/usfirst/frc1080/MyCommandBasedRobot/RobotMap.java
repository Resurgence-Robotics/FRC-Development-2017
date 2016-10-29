// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc1080.MyCommandBasedRobot;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static AnalogGyro driveTraingyro;
    public static CANTalon driveTrainright2;
    public static CANTalon driveTrainright1;
    public static CANTalon driveTrainleft2;
    public static CANTalon driveTrainleft1;
    public static Encoder driveTrainRenc;
    public static Encoder driveTrainLenc;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTraingyro = new AnalogGyro(0);
        LiveWindow.addSensor("DriveTrain", "gyro", driveTraingyro);
        driveTraingyro.setSensitivity(0.007);
        driveTrainright2 = new CANTalon(3);
        LiveWindow.addActuator("DriveTrain", "right2", driveTrainright2);
        
        driveTrainright1 = new CANTalon(2);
        LiveWindow.addActuator("DriveTrain", "right1", driveTrainright1);
        
        driveTrainleft2 = new CANTalon(1);
        LiveWindow.addActuator("DriveTrain", "left2", driveTrainleft2);
        
        driveTrainleft1 = new CANTalon(0);
        LiveWindow.addActuator("DriveTrain", "left1", driveTrainleft1);
        
        driveTrainRenc = new Encoder(2, 3, true, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "Renc", driveTrainRenc);
        driveTrainRenc.setDistancePerPulse(1.0);
        driveTrainRenc.setPIDSourceType(PIDSourceType.kDisplacement);
        driveTrainLenc = new Encoder(0, 1, false, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "Lenc", driveTrainLenc);
        driveTrainLenc.setDistancePerPulse(1.0);
        driveTrainLenc.setPIDSourceType(PIDSourceType.kDisplacement);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}