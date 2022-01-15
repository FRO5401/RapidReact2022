package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


/**
 * Add your docs here.
 */
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro = new AHRS(I2C.Port.kMXP);

  // Configuring Motors
  private WPI_TalonSRX leftDrive1 = new WPI_TalonSRX(RobotMap.DRIVE_MOTOR_LEFT_1);
  private WPI_VictorSPX leftDrive2 = new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_LEFT_2);
  private WPI_VictorSPX leftDrive3 = new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_LEFT_3);

  private WPI_TalonSRX rightDrive1 = new WPI_TalonSRX(RobotMap.DRIVE_MOTOR_RIGHT_1);
  private WPI_VictorSPX rightDrive2 = new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_2);
  private WPI_VictorSPX rightDrive3 = new WPI_VictorSPX(RobotMap.DRIVE_MOTOR_RIGHT_3);



  // Configuring Drives
  private MotorControllerGroup leftDrives = new MotorControllerGroup(leftDrive1, leftDrive2, leftDrive3);
  private MotorControllerGroup rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2, rightDrive3);
  private DifferentialDrive ourDrive = new DifferentialDrive(leftDrives, rightDrives);
  private DifferentialDriveOdometry odometry;

  //PID stuff
  private int loopIndex, slotIndex;
  private double DRIVEBASE_kF = 0;
  private double DRIVEBASE_kP = 0;
  private double DRIVEBASE_kI = 0;
  private double DRIVEBASE_kD = 0;

  private int iaccum = 0;

  // Solenoids Subject to change PneumaticsModuleType
  private Solenoid gearShifter = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.GEAR_SHIFTER);

  // Sensors
  private Encoder leftEncoder = new Encoder(RobotMap.DRIVE_ENC_LEFT_A, RobotMap.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
  private Encoder rightEncoder = new Encoder(RobotMap.DRIVE_ENC_RIGHT_A, RobotMap.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);
  

  public DriveBase() {

    rightDrives.setInverted(true);
    leftDrives.setInverted(true);

    leftEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_RIGHT_DPP);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d());
    
  }

  @Override
  public void periodic() {
    odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  /** Shifts from high gear to low gear */
  public void shiftHighToLow() {
    gearShifter.set(true);
    setDPPLowGear();
  }

  /** Shifts from high gear to low gear */
  public void shiftLowtoHigh() {
    gearShifter.set(false);
    setDPPHighGear();
  }


  /**Sets the DPP to Low Gear */
  public void setDPPLowGear() {
    leftEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.LOW_GEAR_RIGHT_DPP);
  }
  /**Sets the DPP to Low Gear */
  public void setDPPHighGear() {
    leftEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(RobotMap.HIGH_GEAR_RIGHT_DPP);
  }

  /**@return the Pose2D */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**@return The current wheel speeds.*/
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.*/
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, navxGyro.getRotation2d());
  }

  /**Drives the robot
   * @param left the commanded left movement
   * @param right the commanded right movement*/
  public void drive(double left, double right) {
    ourDrive.tankDrive(left, right);
  }

  public void autoTurn(double speed, double angle) {
    double gyroAngle = getGyroAngle();
    if (gyroAngle > (angle+2))
      ourDrive.tankDrive(-speed, speed);
    else if (gyroAngle < (angle-2))
      ourDrive.tankDrive(speed, -speed);
    else 
      ourDrive.tankDrive(0, 0);
  }
  

   // Sets victors to desired speed giving from XboxMove.
   public void autoDrive(double left, double right, double angle) {
    if (left > 0 && right > 0){ //driving forwards
      if (angle > 0){ //drifting right TODO: turn autospeed adjustment and the additional adjustment into 1
        ourDrive.tankDrive(left, right * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.08);
      }
      else if (angle < 0){ //drifting left
        ourDrive.tankDrive(left * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.08, right);
      } 
      else{
        ourDrive.tankDrive(left, right);
      }
    }
    else if (left < 0 && right < 0){ //driving backwards
      if (angle > 0){ //drifting right
        ourDrive.tankDrive(left * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.08, right);
      }
      else if (angle < 0){ //drifting left
        ourDrive.tankDrive(left, right * RobotMap.AUTO_SPEED_ADJUSTMENT * 1.08);
      } 
      else{
        ourDrive.tankDrive(left, right);
      }
    }
    else{ //When leftDrive1 and rightDrive1 are zero
      drive(0,0);      
    }
  }


  /**Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output*/
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrives.setVoltage(leftVolts);
    rightDrives.setVoltage(-rightVolts);
    ourDrive.feed();
  }

  /*Resets the drive encoders to currently read a position of 0.*/
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /** @return the average of the two encoder readings */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /** @return the left drive encoder */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /** @return the right drive encoder */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }



  /**Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   * @param maxOutput the maximum output to which the drive will be constrained*/
  public void setMaxOutput(double maxOutput) {
    ourDrive.setMaxOutput(maxOutput);
  }

  /**Zeroes the heading of the robot.*/
  public void zeroHeading() {
    navxGyro.reset();
  }

  /** @return the robot's heading in degrees, from -180 to 180*/
  public double getHeading() {
    return -navxGyro.getRotation2d().getDegrees();
  }

  // Gets Gyro Angle
  public double getGyroAngle() {
    return navxGyro.getAngle();
  }

  public void resetGyroAngle() {
    navxGyro.reset();
  }

  // For autonomous driving
  public double getEncoderDistance(int encoderNumber) {
    double leftDistAdj = leftDrive1.getSelectedSensorPosition();
    double rightDistAdj = rightDrive1.getSelectedSensorPosition();
    double avgDistance = ((-1 * leftDistAdj) + rightDistAdj) / 2;

    if (encoderNumber == 1) {
      return leftDistAdj;
    } else if (encoderNumber == 2) {
      return rightDistAdj;
    } else {
      return avgDistance;
    }
  }

  /** @return The turn rate of the robot, in degrees per second*/
  public double getTurnRate() {
    return -navxGyro.getRate();
  }
}