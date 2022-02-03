package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Controls;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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

  // Solenoids Subject to change PneumaticsModuleType
  private Solenoid gearShifter;

  // Sensors
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  

  public DriveBase() {

    //Instaniate some drivebase stuff
    navxGyro = new AHRS(I2C.Port.kMXP);
    leftDrive1 = new WPI_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1);
    leftDrive2 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_2);
    leftDrive3 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_3);
    rightDrive1 = new WPI_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_1);
    rightDrive2 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_2);
    rightDrive3 = new WPI_VictorSPX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_3);
    leftDrives = new MotorControllerGroup(leftDrive1, leftDrive2, leftDrive3);
    rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2, rightDrive3);
    ourDrive = new DifferentialDrive(leftDrives, rightDrives);
    gearShifter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    leftEncoder = new Encoder(Constants.ControlConstants.DRIVE_ENC_LEFT_A, Constants.ControlConstants.DRIVE_ENC_LEFT_B, true, EncodingType.k4X);
    rightEncoder = new Encoder(Constants.ControlConstants.DRIVE_ENC_RIGHT_A, Constants.ControlConstants.DRIVE_ENC_RIGHT_B, false, EncodingType.k4X);

    leftDrives.setInverted(true);
    ourDrive.setExpiration(0.1);
    ourDrive.setMaxOutput(1.0);
    leftDrive1.setNeutralMode(NeutralMode.Brake);
    leftDrive2.setNeutralMode(NeutralMode.Brake);
    leftDrive3.setNeutralMode(NeutralMode.Brake);
    rightDrive1.setNeutralMode(NeutralMode.Brake);
    rightDrive2.setNeutralMode(NeutralMode.Brake);
    rightDrive3.setNeutralMode(NeutralMode.Brake);
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_RIGHT_DPP);

    odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d());
    competitionTab = Shuffleboard.getTab("Competition");
    programmerTab = Shuffleboard.getTab("Programming");
  }

  @Override
  public void periodic() {
    odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    reportSensors();
    SmartDashboard.putNumber("Axis", Controls.xboxAxis(Controls.driver, "LS-X"));
   // drivebaseShuffleboard();
  }

  //Shift from high gear to low gear
  public void shiftHighToLow() {
    //gearShifter.set(true);
    setDPPLowGear();
  }

  //Shift from low gear to high gear
  public void shiftLowtoHigh() {
    gearShifter.set(false);
    setDPPHighGear();
  }


  //Set the DPP to low gear
  public void setDPPLowGear() {
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(Constants.DriveConstants.LOW_GEAR_RIGHT_DPP);
  }
  //Set the DPP to high gear
  public void setDPPHighGear() {
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.HIGH_GEAR_LEFT_DPP);
    rightEncoder.setDistancePerPulse(Constants.DriveConstants.HIGH_GEAR_RIGHT_DPP);
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

  //Separate method to call the generic tank drive method
  public void drive(double left, double right) {
    ourDrive.tankDrive(left, right);
  }

  //Automatic turning method
  public void autoTurn(double speed, double angle) {
    double gyroAngle = getGyroAngle();
    if (gyroAngle > (angle+2))
      ourDrive.tankDrive(-speed, speed);
    else if (gyroAngle < (angle-2))
      ourDrive.tankDrive(speed, -speed);
    else 
      ourDrive.tankDrive(0, 0);
  }

  //Experimental turning method
  public void autoVisionTurn(double speed) {
    drive(speed, -speed);
    System.out.println(speed);
  }
  

   //For driving automatically
   public void autoDrive(double left, double right, double angle) {
    if (left > 0 && right > 0){ //driving forwards
      if (angle > 0){ //drifting right TODO: turn autospeed adjustment and the additional adjustment into 1
        ourDrive.tankDrive(left, right * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08);
      }
      else if (angle < 0){ //drifting left
        ourDrive.tankDrive(left * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08, right);
      } 
      else{
        ourDrive.tankDrive(left, right);
      }
    }
    else if (left < 0 && right < 0){ //driving backwards
      if (angle > 0){ //drifting right
        ourDrive.tankDrive(left * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08, right);
      }
      else if (angle < 0){ //drifting left
        ourDrive.tankDrive(left, right * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08);
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
    //leftEncoder.reset();
    //rightEncoder.reset();
  }

  public void resetTalon(boolean input) {
    leftDrive1.configClearPositionOnQuadIdx(input, 10000);
    rightDrive1.configClearPositionOnQuadIdx(input, 10000);
  }

  //Get the average encoder distance
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  //Get the left encoder's value
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  //Get the right encoder's value
  public Encoder getRightEncoder() {
    return rightEncoder;
  }



  //Set the max voltage output
  public void setMaxOutput(double maxOutput) {
    ourDrive.setMaxOutput(maxOutput);
  }

  //resets the gyro
  public void zeroHeading() {
    navxGyro.reset();
  }

  //Get the 2D positioning for the gyro
  public double getHeading() {
    return -navxGyro.getRotation2d().getDegrees();
  }

  // Gets Gyro Angle
  public double getGyroAngle() {
    return navxGyro.getAngle();
  }

  public double getGyroYaw(){
    return navxGyro.getYaw();
  }

  public double getGyroPitch(){
    return navxGyro.getPitch();
  }

  public double getGyroRoll(){
    return navxGyro.getRoll();
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

  public double getTalonDistance(){

    return leftDrive1.getSelectedSensorPosition() * Constants.DriveConstants.LOW_GEAR_LEFT_DPP;
  }

  //Get the turn rate/angle per second of the gyro
  public double getTurnRate() {
    return -navxGyro.getRate();
  }

  public double getAverageMotorVelocity(){
    return (Math.abs(leftDrive1.getSelectedSensorVelocity())+Math.abs(rightDrive1.getSelectedSensorVelocity()))/2;
  }

  public double getLeftVelocity() {
    return leftDrive1.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return rightDrive1.getSelectedSensorVelocity();
  }

  public double getLeftPosition(){
    return leftDrive1.getSelectedSensorPosition();
  }

  public double getRightPosition(){
    return rightDrive1.getSelectedSensorPosition();
  }

  //Report the values
  public void reportSensors() {
    SmartDashboard.putNumber("Gyro Rotations", getGyroAngle()/360);
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle()/1);
    SmartDashboard.putNumber("Gyro Yaw", getGyroYaw());
    SmartDashboard.putNumber("Gyro Pitch", getGyroPitch());
    SmartDashboard.putNumber("Gyro Roll", getGyroRoll());
    SmartDashboard.putNumber("Gyro Turn Rate", getTurnRate());
    SmartDashboard.putNumber("Left Motors Speed", getLeftVelocity());
    SmartDashboard.putNumber("Right Motors Speed", getRightVelocity());
    SmartDashboard.putNumber("Left Motor Position", getLeftPosition());
    SmartDashboard.putNumber("Right Motor Position", getRightPosition());
  }

  //Thing that does not work TODO: make work
  public void drivebaseShuffleboard(){
    competitionTab.add("Robot Speed",getAverageMotorVelocity());
    programmerTab.add("Left Motor Speed",leftDrive1.getSelectedSensorVelocity())
        .withWidget(BuiltInWidgets.kGraph);
    competitionTab.add("Right Motor Speed",rightDrive1.getSelectedSensorVelocity())
        .withWidget(BuiltInWidgets.kGraph);   
  }
}