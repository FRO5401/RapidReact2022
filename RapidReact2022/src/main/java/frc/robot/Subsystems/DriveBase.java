package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Controls;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

//TODO: Fix Drivebase encoder logic and auto logic
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro;
  private ShuffleboardTab competitionTab;
  private ShuffleboardTab programmerTab;

  // Configuring Motors
  private WPI_TalonSRX leftDrive1;
  private WPI_VictorSPX leftDrive2;
  private WPI_VictorSPX leftDrive3;

  private WPI_TalonSRX rightDrive1;
  private WPI_VictorSPX rightDrive2;
  private WPI_VictorSPX rightDrive3;


  // Configuring Drives
  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive ourDrive;
  private DifferentialDriveOdometry odometry;

  //PID stuff
  private int loopIndex, slotIndex;
  private double DRIVEBASE_kF = 0;
  private double DRIVEBASE_kP = 0;
  private double DRIVEBASE_kI = 0;
  private double DRIVEBASE_kD = 0;

  private int iaccum = 0;

  // Solenoid
  private Solenoid gearShifter;

  // Sensors
  private Encoder leftEncoder;
  private Encoder rightEncoder;

  public DriveBase() {

    //Instantating the physical parts on the drivebase
    navxGyro = new AHRS(I2C.Port.kMXP);
    leftDrive1 = new WPI_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1);
    leftDrive2 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_2);
    leftDrive3 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_3);
    rightDrive1 = new WPI_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_1);
    rightDrive2 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_2);
    rightDrive3 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_3);
    leftEncoder = new Encoder(Constants.ControlConstants.DRIVE_ENC_LEFT_A, Constants.ControlConstants.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
    rightEncoder = new Encoder(Constants.ControlConstants.DRIVE_ENC_RIGHT_A, Constants.ControlConstants.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);

    //Organization of those physical parts
    leftDrives = new MotorControllerGroup(leftDrive1, leftDrive2, leftDrive3);
    rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2, rightDrive3);
    ourDrive = new DifferentialDrive(leftDrives, rightDrives);
    gearShifter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    //Configuring parts
    rightDrives.setInverted(true);
    ourDrive.setExpiration(0.1);
    ourDrive.setMaxOutput(1.0);
    setDrivebaseNeutralMode(NeutralMode.Brake);
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_RIGHT_DPP);

    //Shuffleboard and Path Planning
    odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d());
    competitionTab = Shuffleboard.getTab("Competition");
    programmerTab = Shuffleboard.getTab("Programming");
  }

  //Report sensors whenever
  @Override
  public void periodic() {
    odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    reportSensors();

    //Purely for testing purposes
    SmartDashboard.putNumber("Axis", Controls.xboxAxis(Controls.driver, "LS-X"));
   // drivebaseShuffleboard();
  }

  //Shift gears
  public void shift(String gear) {
    if(gear.toUpperCase().equals("LOW")) 
      gearShifter.set(true);
    else if(gear.toUpperCase().equals("HIGH")) 
      gearShifter.set(false);
    DPPShifter(gear);  
  }  

  //Update DPP to be compliant with gears
  public void DPPShifter(String gear) {
    if(gear.toUpperCase().equals("LOW")) {
      leftEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_LEFT_DPP);
      rightEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_RIGHT_DPP);
    } else if(gear.toUpperCase().equals("HIGH")) {
      leftEncoder.setDistancePerPulse(Constants.DriveConstants.HIGH_GEAR_LEFT_DPP);
      rightEncoder.setDistancePerPulse(Constants.DriveConstants.HIGH_GEAR_RIGHT_DPP);
    }
  } 

  //Separate method to call the generic tank drive method
  public void drive(double left, double right) {
    ourDrive.tankDrive(left, right);
  }

  //Automatic turning method
  public void autoTurn(double speed, double angle) {
    double gyroAngle = getGyroAngle();
    if (gyroAngle > (angle+2))
      drive(-speed, speed);
    else if (gyroAngle < (angle-2))
      drive(speed, -speed);
    else 
      drive(0, 0);
  }

  //Experimental turning method
  public void autoVisionTurn(double speed) {
    drive(speed, -speed);
    System.out.println(speed);
  }
  

   //For driving automatically
   public void autoDrive(double left, double right, double angle) {
    if (left > 0 && right > 0){ //driving forwards
      drive(
        angle < 0 ? left : left * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08,
        angle > 0 ? right : right * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08
      );
    }
    else if (left < 0 && right < 0){ //driving backwards
      drive(
        angle > 0 ? left : left * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08,
        angle < 0 ? right : right * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08
      );
    }
    else{ //When leftDrive1 and rightDrive1 are zero
      drive(0,0);      
    }
  }

  //Set the idle mode of the drivebase, should be brake normally
  public void setDrivebaseNeutralMode(NeutralMode mode){
    leftDrive1.setNeutralMode(mode);
    leftDrive2.setNeutralMode(mode);
    leftDrive3.setNeutralMode(mode);
    rightDrive1.setNeutralMode(mode);
    rightDrive2.setNeutralMode(mode);
    rightDrive3.setNeutralMode(mode);
  }

  //Get the 2D position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  //Get the wheel speeds returned by the differential drive
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  //Odometry reseter
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, navxGyro.getRotation2d());
  }

  //Controls the left and right sides of the drive directly with voltages.
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrives.setVoltage(leftVolts);
    rightDrives.setVoltage(-rightVolts);
    ourDrive.feed();
  }

  //Resets the drive encoders
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }


  //Get the average encoder distances, regular encoder distances, encoders themselves, and distance
  public double getAverageEncoderDistance() { return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0; }
  public Encoder getLeftEncoder() { return leftEncoder; }
  public Encoder getRightEncoder() { return rightEncoder; }
 

  //Set the max voltage output
  public void setMaxOutput(double maxOutput) { ourDrive.setMaxOutput(maxOutput); }

  //Gets/Resets the Gyro Angles
  public double getGyroAngle() { return navxGyro.getAngle(); }
  public double getGyroYaw(){ return navxGyro.getYaw(); }
  public double getGyroPitch(){ return navxGyro.getPitch(); }
  public double getGyroRoll(){ return navxGyro.getRoll(); }
  public void resetGyroAngle() { navxGyro.reset(); }
  
  //For path planning
  public void zeroHeading() { navxGyro.reset(); }
  public double getHeading() { return -navxGyro.getRotation2d().getDegrees(); }
  public double getTurnRate() { return -navxGyro.getRate();}


  //Report the values
  public void reportSensors() {
    SmartDashboard.putNumber("Gyro Rotations", getGyroAngle()/360);
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Gyro Yaw", getGyroYaw());
    SmartDashboard.putNumber("Gyro Pitch", getGyroPitch());
    SmartDashboard.putNumber("Gyro Roll", getGyroRoll());
    SmartDashboard.putNumber("Gyro Turn Rate", getTurnRate());
  }

  //Thing that does not work TODO: make work
  public void drivebaseShuffleboard(){
      
  }
}