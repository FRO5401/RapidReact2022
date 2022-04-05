package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Controls;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import static frc.robot.Tabs.*;

//TODO: Remember to bring gear shifter back
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro;
  
  // Configuring Motors
  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;

  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;

  // Configuring Drives
  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive ourDrive;
  private DifferentialDriveOdometry odometry;

  private boolean compressorState = true;

  //PID stuff
  //private int loopIndex, slotIndex;
  private double kFF = 1;
  private double kP = 34.199/2048;
  private double kI = 0;//1;
  private double kD = 0;//1.5784;
  private SparkMaxPIDController leftDrive1PidController;
  private SparkMaxPIDController leftDrive2PidController;

  private SparkMaxPIDController rightDrive1PidController;
  private SparkMaxPIDController rightDrive2PidController;

  //private int iaccum = 0;

  // Solenoid
  private Solenoid gearShifter;

  // Sensors
  private RelativeEncoder leftEncoders[];
  private RelativeEncoder rightEncoders[];
  public Compressor compressor;
  private PowerDistribution pdp;

  private double overCurrentTime;
  private boolean overCurrentFlag;

  public DriveBase() {

    //Instantating the physical parts on the drivebase
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    pdp = new PowerDistribution();
    navxGyro = new AHRS(SPI.Port.kMXP);
    leftDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_2, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_1, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_2, MotorType.kBrushless);

    leftDrive1PidController = leftDrive1.getPIDController();
    leftDrive2PidController = leftDrive2.getPIDController();
    rightDrive1PidController = rightDrive1.getPIDController();
    rightDrive2PidController = rightDrive2.getPIDController();

    leftEncoders = new RelativeEncoder[2];
    rightEncoders = new RelativeEncoder[2];
    leftEncoders[0] = leftDrive1.getEncoder();
    leftEncoders[1] = leftDrive2.getEncoder();
    rightEncoders[0] = rightDrive1.getEncoder();
    rightEncoders[1] = rightDrive2.getEncoder();
    rightDrive1.setSmartCurrentLimit(55, 20);
    rightDrive2.setSmartCurrentLimit(55, 20);
    leftDrive1.setSmartCurrentLimit(55, 20);
    leftDrive2.setSmartCurrentLimit(55, 20);
    leftDrive1.setInverted(true);
    leftDrive2.setInverted(true);

    leftDrive1PidController.setP(kP);
    leftDrive1PidController.setI(kI);
    leftDrive1PidController.setD(kD);
    leftDrive1PidController.setFF(kFF);
    leftDrive2PidController.setP(kP);
    leftDrive2PidController.setI(kI);
    leftDrive2PidController.setD(kD);
    leftDrive2PidController.setFF(kFF);
    rightDrive1PidController.setP(kP);
    rightDrive1PidController.setI(kI);
    rightDrive1PidController.setD(kD);
    rightDrive1PidController.setFF(kFF);
    rightDrive2PidController.setP(kP);
    rightDrive2PidController.setI(kI);
    rightDrive2PidController.setD(kD);
    rightDrive2PidController.setFF(kFF);
    //Organization of those physical parts
    leftDrives = new MotorControllerGroup(leftDrive1, leftDrive2);
    rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2);
    
    leftEncoders[0].setPositionConversionFactor(100/21);
    leftEncoders[1].setPositionConversionFactor(100/21);
    rightEncoders[0].setPositionConversionFactor(100/21);
    rightEncoders[1].setPositionConversionFactor(100/21);

    ourDrive = new DifferentialDrive(leftDrives, rightDrives);
    gearShifter = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.DriveConstants.GEAR_SHIFTER);

    //Configuring parts
    //leftDrives.setInverted(true);
    //ourDrive.setExpiration(0.1);
    //ourDrive.setMaxOutput(1.0);
    setDrivebaseIdleMode(IdleMode.kBrake);
    compressor.enableDigital();

    //Shuffleboard and Path Planning
    odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d());
    drivebaseShuffleboard();
    shift("LOW");
    navxGyro.calibrate();
  }

  //Report sensors whenever
  @Override
  public void periodic() {
  // odometry.update(navxGyro.getRotation2d(), leftEncoders[0].getPosition(), rightEncoders[0].getPosition());
    reportSensors();
    
 
  }

  //Compressor control
  public void compressorToggle(){
    compressorState = !compressorState;
    setCompressor(compressorState);
  }

  //Set the Compressor
  public void setCompressor(boolean state){
    if (state == false)
      compressor.disable();
    else
      compressor.enableDigital();  
  }

  //Shift gears
  public void shift(String gear) {
    if(gear.toUpperCase().equals("LOW")) 
      gearShifter.set(false);
    else if(gear.toUpperCase().equals("HIGH")) 
      gearShifter.set(true);
    DPPShifter(gear);  
  }  

  public boolean getGear(){ return gearShifter.get();}

  //Update DPP to be compliant with gears
  public void DPPShifter(String gear) {
    if(gear.toUpperCase().equals("LOW")) {
      leftEncoders[0].setPositionConversionFactor(Constants.DriveConstants.LOW_GEAR_LEFT_DPP);
      leftEncoders[1].setPositionConversionFactor(Constants.DriveConstants.LOW_GEAR_LEFT_DPP);
      rightEncoders[0].setPositionConversionFactor(Constants.DriveConstants.LOW_GEAR_RIGHT_DPP);
      rightEncoders[1].setPositionConversionFactor(Constants.DriveConstants.LOW_GEAR_RIGHT_DPP);
    } else if(gear.toUpperCase().equals("HIGH")) {
      leftEncoders[0].setPositionConversionFactor(Constants.DriveConstants.HIGH_GEAR_LEFT_DPP);
      leftEncoders[1].setPositionConversionFactor(Constants.DriveConstants.HIGH_GEAR_LEFT_DPP);
      rightEncoders[0].setPositionConversionFactor(Constants.DriveConstants.HIGH_GEAR_RIGHT_DPP);
      rightEncoders[1].setPositionConversionFactor(Constants.DriveConstants.HIGH_GEAR_RIGHT_DPP);
    }
  } 

  //Separate method to call the generic tank drive method
  public void drive(double left, double right) {
    ourDrive.tankDrive(left, right);
  }

  public void overCurrentLimit() {
    if (getPDPCurrent() > 180 && Timer.getFPGATimestamp() - overCurrentTime > .1) {
      overCurrentFlag = true;
      overCurrentTime = Timer.getFPGATimestamp();
      double newlimit = getPDPCurrent()-120;
      newlimit /= 4;
      newlimit = 55 - newlimit;
      rightDrive1.setSmartCurrentLimit((int)newlimit, 20);
      rightDrive2.setSmartCurrentLimit((int)newlimit, 20);
      leftDrive1.setSmartCurrentLimit((int)newlimit, 20);
      leftDrive2.setSmartCurrentLimit((int)newlimit, 20);;
    }
    else if (Timer.getFPGATimestamp() - overCurrentTime > 1 && overCurrentFlag){
      overCurrentFlag = false;
      rightDrive1.setSmartCurrentLimit(55, 20);
      rightDrive2.setSmartCurrentLimit(55, 20);
      leftDrive1.setSmartCurrentLimit(55, 20);
      leftDrive2.setSmartCurrentLimit(55, 20);
    }
  }

  public void driveToPos(double distance) {
    leftDrive1PidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    leftDrive2PidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    rightDrive1PidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    rightDrive2PidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    
  }

  //Automatic turning method
  public void autoTurn(double speed, double angle) {
    double gyroAngle = getGyroYaw();
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
    //System.out.println(speed);
  }
  

   //For driving automatically
   public void autoDrive(double left, double right, double angle) {
     
    if (left > 0 && right > 0){ //driving forwards
      drive(
        angle > 0 ? left : left * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08,
        angle < 0 ? right : right * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08
      );
    }
    else if (left < 0 && right < 0){ //driving backwards
      drive(
        angle < 0 ? left : left * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08,
        angle > 0 ? right : right * Constants.AutoConstants.AUTO_SPEED_ADJUSTMENT * 1.08
      );
    }
    else{ //When leftDrive1 and rightDrive1 are zero
      drive(0,0);      
    }
  }

  //Set the idle mode of the drivebase, should be brake normally
  public void setDrivebaseIdleMode(IdleMode mode){
    leftDrive1.setIdleMode(mode);
    leftDrive2.setIdleMode(mode);
    rightDrive1.setIdleMode(mode);
    rightDrive2.setIdleMode(mode);
  }

  //Get the 2D position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  //Get the wheel speeds returned by the differential drive
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoders[0].getVelocity(), rightEncoders[0].getVelocity());
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

  //Resets the spark motor controllers
  public void resetEncoders() {
    for (int i = 0; i < leftEncoders.length; i++) {
      leftEncoders[i].setPosition(0);
     rightEncoders[i].setPosition(0);
    }
  }

  //Get the average encoder distances, regular encoder distances, encoders themselves, and distance
  public double getAverageEncoderDistance() { return (Math.abs(leftEncoders[0].getPosition()+leftEncoders[1].getPosition())/2+Math.abs(rightEncoders[0].getPosition()+rightEncoders[1].getPosition())/2)/2; }
  public double getLeftEncodersDistance(int encoderNumber) { return leftEncoders[encoderNumber].getPosition(); }
  public double getRightEncodersDistance(int encoderNumber) {return rightEncoders[encoderNumber].getPosition(); }
  public RelativeEncoder getLeftEncoder(int encoderNumber) { return leftEncoders[encoderNumber]; }
  public RelativeEncoder getRightEncoder(int encoderNumber) { return rightEncoders[encoderNumber]; }
  public double getPosition(){ return leftEncoders[0].getPosition(); }

  //Get velocities
  public double getAverageMotorVelocity(){ return (Math.abs(leftEncoders[0].getVelocity())+Math.abs(rightEncoders[0].getVelocity()))/2; }
  public double getLeftVelocity() { return leftEncoders[0].getVelocity(); }
  public double getRightVelocity() { return rightEncoders[0].getVelocity(); }

  //Set the max voltage output
  public void setMaxOutput(double maxOutput) { ourDrive.setMaxOutput(maxOutput); }

  //Gets/Resets the Gyro Angles
  public double getGyroAngle() { return navxGyro.getAngle(); }
  public double getGyroYaw(){ return navxGyro.getYaw(); }
  public double getGyroPitch(){ return navxGyro.getPitch(); }
  public double getGyroRoll(){ return navxGyro.getRoll();}
  public void resetGyroAngle() {
     navxGyro.reset(); 
  }
  
  //For path planning
  public void zeroHeading() { navxGyro.reset(); }
  public double getHeading() { return -navxGyro.getRotation2d().getDegrees(); }
  public double getTurnRate() { return -navxGyro.getRate();}

  //Compressor stuff
  public double getPressure() { return compressor.getPressure();}
  public boolean getPressureStatus(){
    if(compressor.getPressure() <= 10.0){
      return true;
    }
    else{
      return false;
    }
  }
  public double getCompressorCurrent() { return compressor.getCurrent(); }
  public double getCompressorVoltage() { return compressor.getAnalogVoltage(); }
  public boolean getEnabled() { return compressor.enabled(); }
  public boolean getSwitchValue() { return compressor.getPressureSwitchValue(); } // true if low pressure

  //PDP stuff
  public double getPDPCurrent() { return pdp.getTotalCurrent(); }
  public double getSpecificPDPCurrent(int channel) { return pdp.getCurrent(channel); }
  public double getPDPVoltage() { return pdp.getVoltage(); }
  public double getPDPPower() { return pdp.getTotalPower(); }
  public double getPDPEnergy() { return pdp.getTotalEnergy(); }
  public double getPDPTemperature() { return pdp.getTemperature(); }
  
  //Report the values by updating shuffleboard
  public void reportSensors() {
    //System.out.println(navxGyro.getBoardYawAxis());

    //Graph config
    
    //System.out.println(Controls.xboxAxis(Controls.driver, "RT").getAxis());

    //Competition config
    if(Constants.SubsystemConstants.shuffleboardCompMode[1]){
      
        shifterComp.setBoolean(getGear());
    }
    else{
        speedGraph.setDouble(getAverageMotorVelocity());
      leftSpeedGraph.setDouble(getLeftVelocity());
      rightSpeedGraph.setDouble(getRightVelocity());
      leftPositionGraph.setDouble(getLeftEncodersDistance(0));
      rightPositionGraph.setDouble(getRightEncodersDistance(0));
      //turnRateGraph.setDouble(getTurnRate());

      //Testing config
      speedEntry.setDouble(getAverageMotorVelocity());
      leftSpeedEntry.setDouble(getLeftVelocity());
      rightSpeedEntry.setDouble(getRightVelocity());
      leftPositionEntry.setDouble(getLeftEncodersDistance(0));
      rightPositionEntry.setDouble(getRightEncodersDistance(0));
      rotationsEntry.setDouble(getGyroAngle()/360);
      angleEntry.setDouble(getGyroAngle());
      yawEntry.setDouble(getGyroYaw());
      pitchEntry.setDouble(getGyroPitch());
      rollEntry.setDouble(getGyroRoll());
      turnRateEntry.setDouble(getTurnRate());
      shifterEntry.setBoolean(getGear());
      axisTester.setDouble(Controls.xboxAxis(Controls.driver, "RT").getAxis());
    }
  
  }

  //Shuffleboard config
  public void drivebaseShuffleboard(){
    //Graph config

    if(!Constants.SubsystemConstants.shuffleboardCompMode[1]){
        speedGraph = graphTab.add("Robot Speed",getAverageMotorVelocity())
        .withWidget(BuiltInWidgets.kGraph).getEntry();
      leftSpeedGraph = graphTab.add("Left Motor Speed",getLeftVelocity())
          .withWidget(BuiltInWidgets.kGraph).getEntry();
      rightSpeedGraph = graphTab.add("Right Motor Speed",getRightVelocity())
          .withWidget(BuiltInWidgets.kGraph).getEntry(); 
      leftPositionGraph = graphTab.add("Left Motor Position",getLeftEncodersDistance(0))
          .withWidget(BuiltInWidgets.kGraph).getEntry();     
      rightPositionGraph = graphTab.add("Right Motor Position",getRightEncodersDistance(0))
          .withWidget(BuiltInWidgets.kGraph).getEntry();   
      /*turnRateGraph = graphTab.add("Gyro Turn Rate", getTurnRate())
          .withWidget(BuiltInWidgets.kGraph).getEntry();    */
      
      //Testing Tab
      
      speedEntry = testingTab.add("Robot Speed",getAverageMotorVelocity()).getEntry();
      leftSpeedEntry = testingTab.add("Left Motor Speed",getLeftVelocity()).getEntry();
      rightSpeedEntry = testingTab.add("Right Motor Speed",getRightVelocity()).getEntry(); 
      leftPositionEntry = testingTab.add("Left Motor Position",getLeftEncodersDistance(0)).getEntry();     
      rightPositionEntry = testingTab.add("Right Motor Position",getRightEncodersDistance(0)).getEntry();  
      rotationsEntry = testingTab.add("Gyro Rotations", getGyroAngle()/360).getEntry();
      angleEntry = testingTab.add("Gyro Angle", getGyroAngle()).getEntry();
      yawEntry = testingTab.add("Gyro Yaw", getGyroYaw()).getEntry();
      pitchEntry = testingTab.add("Gyro Pitch", getGyroPitch()).getEntry();
      rollEntry = testingTab.add("Gyro Roll", getGyroRoll()).getEntry();
      turnRateEntry = testingTab.add("Gyro Turn Rate", getTurnRate()).getEntry();
      shifterEntry = testingTab.add("Solenoid Gear", getGear()).getEntry();
      
      axisTester = graphTab.add("Axis", Controls.xboxAxis(Controls.driver, "RT").getAxis()).withWidget(BuiltInWidgets.kGraph).getEntry();
      
      shifterComp = competitionTab.add("High Gear", getGear()).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    //Competition Tab

    }
    else{
      shifterComp = competitionTab.add("High Gear", getGear()).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    }
  }
}