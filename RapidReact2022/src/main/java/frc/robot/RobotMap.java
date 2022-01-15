package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class RobotMap {
    /*** Constants ***/
  // OI
  public static final double AXIS_THRESHOLD = 0.25;

  // DriveBase
  public static final double TELEOP_SPEED_ADJUSTMENT_RIGHT = 0.99;
  public static final double TELEOP_SPEED_ADJUSTMENT_LEFT = 1;
  public static final double LOW_GEAR_LEFT_DPP = 0.1466004558282468; // Low gear skews left
  public static final double LOW_GEAR_RIGHT_DPP  = 0.1568175312974026;
  public static final double HIGH_GEAR_LEFT_DPP = 0.1568175312974026; // High gear skews left
  public static final double HIGH_GEAR_RIGHT_DPP = 0.1466004558282468;
  public static final int DRIVEBASE_THRESHOLD_FOR_PID = 0;

  // XboxMove
  public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
  public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
  public static final double SPIN_SENSITIVITY = 0.8;

  /*** Operator Interfaces ***/
  // Controllers
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

  /*** Motors ***/
  public static final int SMART_MOTOR = 0;
  public static final int REGULAR_MOTOR_1 = 1;
  public static final int REGULAR_MOTOR_2 = 2;


  /*** Solenoids (Single and Double) ***/
  // DoubleSolenoids have an IN and an OUT constant.
  // Solenoids have just one constant.
  // PCM (Pneumatic Control Module)
  public static final int PCM_ID = 0;
  public static final int DOUBLE_SOLENOID_1_PORT_1 = 1;
  public static final int DOUBLE_SOLENOID_1_PORT_2 = 2;
  public static final int SOLENOID_1 = 3;
  public static final int SOLENOID_2 = 4; 


  /*** Sensors ***/
  // Encoders
  public static final int DRIVE_ENC_LEFT_A = 3;
  public static final int DRIVE_ENC_RIGHT_A = 1;
  public static final int DRIVE_ENC_LEFT_B = 4;
  public static final int DRIVE_ENC_RIGHT_B = 2;

  /*** Autonomous ***/
  public static final double ANGLE_THRESHOLD = 6;
  public static final double AUTO_SPEED_ADJUSTMENT = 1.4;
  public static final double AUTO_TURN_SPEED = 0.8;
  public static final double AUTO_TURN_PRECISION = 0.5;

  // Trajectory Constants
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
}
