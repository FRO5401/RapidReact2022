package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {

  public class DriveConstants {
    //True Constants
    public static final double TELEOP_SPEED_ADJUSTMENT_RIGHT = 0.99;
    public static final double TELEOP_SPEED_ADJUSTMENT_LEFT = 1;
    public static final double LOW_GEAR_LEFT_DPP = 1; // Low gear skews left
    public static final double LOW_GEAR_RIGHT_DPP  = 1;
    public static final double HIGH_GEAR_LEFT_DPP = 1; // High gear skews left
    public static final double HIGH_GEAR_RIGHT_DPP = 1;
    public static final int DRIVEBASE_THRESHOLD_FOR_PID = 0;

    ////DRIVE MOTORS////
   //Right Motors
    public static final int DRIVE_MOTOR_RIGHT_1 = 1;
    public static final int DRIVE_MOTOR_RIGHT_2 = 3;

    //Left Motors
    public static final int DRIVE_MOTOR_LEFT_1 = 2;
    public static final int DRIVE_MOTOR_LEFT_2 = 4;

    public static final int GEAR_SHIFTER = 0;
  }

  public class ControlConstants {
    //Operator Interfaces
    public static final double AXIS_THRESHOLD = 0.25;
    public static final int XBOX_CONTROLLER_DRIVER = 0;
    public static final int XBOX_CONTROLLER_OPERATOR = 1;

     // Buttons
    public static final int XBOX_BUTTON_A = 1;
    public static final int XBOX_BUTTON_B = 2;
    public static final int XBOX_BUTTON_X = 3;
    public static final int XBOX_BUTTON_Y = 4;
    public static final int XBOX_BUTTON_LEFT_BUMPER = 5;
    public static final int XBOX_BUTTON_RIGHT_BUMPER = 6;
    public static final int XBOX_BUTTON_BACK = 7;
    public static final int XBOX_BUTTON_START = 8;
    public static final int XBOX_BUTTON_L3 = 9;
    public static final int XBOX_BUTTON_R3 = 10;

    // Axes
    public static final int XBOX_AXIS_LEFT_X = 0;
    public static final int XBOX_AXIS_LEFT_Y = 1;
    public static final int XBOX_AXIS_LEFT_TRIGGER = 2;
    public static final int XBOX_AXIS_RIGHT_TRIGGER = 3;
    public static final int XBOX_AXIS_RIGHT_X = 4;
    public static final int XBOX_AXIS_RIGHT_Y = 5;

    //Drive Sensitivities
    public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
    public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
    public static final double SPIN_SENSITIVITY = 0.8;

    //public static final int PCM_ID = 0;

    //Solenoids


    ////Sensors////
    //Encoders
    public static final int DRIVE_ENC_LEFT_A = 3;
    public static final int DRIVE_ENC_RIGHT_A = 1;
    public static final int DRIVE_ENC_LEFT_B = 4;
    public static final int DRIVE_ENC_RIGHT_B = 2;
  }

  public static class AutoConstants {
    //True Constants
    public static final double ANGLE_THRESHOLD = 6;
    public static final double AUTO_SPEED_ADJUSTMENT = 1.05;
    public static final double AUTO_TURN_SPEED = 0.8;
    public static final double AUTO_TURN_PRECISION = 0.5;
    public static final double ANGULAR_THRESHOLD = 2;

    //Trajectory Constants
    public static final double kTrackwidthMeters = 0.6858;
    public static final double ksVolts = 1.23;
    public static final double kvVoltSecondsPerMeter = 0.11;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0235;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kPDriveVel = 1.05;
    public static final double SCUFFED_CORRECTION_CONSTANT = 59/109.22;
  }

  public static class SubsystemConstants {
    //Infeed
    public static final double INFEED_MOTOR_SPEED = 0.3;
    public static final int INFEED_GATE = 1;
    public static final int INFEED_SPARK_2 = 5; //right
    public static final int INFEED_SPARK_1 = 6; //left
    

    //Internal Mechanism
    public static final int INTERNAL_MECH_MOTOR = 7; //Right
    public static final int INTERNAL_MECH_MOTOR2 = 15; //Left
    public static final double MECH_SPEED = 0.95;

    //Shooter
    public static final int SHOOTER_MOTOR_1 = 9; //Left
    public static final int SHOOTER_MOTOR_2 = 10; //Right
    public static final double SHOOTER_SPEED = 0.95; //0.75 for blue from the line radially*** 
    public static final double LOADER_SPEED = 0.55;
	  public static final int BALL_LOADER = 8;
    public static final double SHOOTER_WAIT_TIME = 1.75;

    //Shooter PID Constants 
     
    public static final double kS = 0.69587;
    public static final double kV = 1.5559; 
    public static final double kA = 0.081132;
    public static final double shootLowSpeed = 15000;
    public static final double shootHighSpeed = 15000;
    public static final double feedFordwardConstant = 0.9;

    //Climber
    public static final int TRANS_MOTOR_1 = 11;
    public static final int TRANS_MOTOR_2 = 12;
    public static final int ROTATE_MOTOR_1 = 13;
    public static final int ROTATE_MOTOR_2 = 14;
    public static final int RATCHET_SOLENOID = 2;
    public static final int DIGITAL_INPUT_1 = 0;
    public static final int DIGITAL_INPUT_2 = 0;
     
    //climber angle calculations
    public static int measuredHorizontalPosition = 840;
    public static double ticksPerDegree = (4096 / 360); 
    public static double gravityFF = 0.07; //predetermined gravity feedforward constant
    public static double climberArmLength = 0;
    public static double robotFrontOffset = 0;
    public static double robotBackOffset = 0;
    public static int extensionLimit = 16;
    
  }
     
}
