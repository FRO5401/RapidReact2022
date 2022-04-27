package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;
import static frc.robot.Tabs.*;

public class Shooter extends SubsystemBase{
 
    private double maxVelocity = 16220;
    private WPI_TalonFX shooterMotor1;
    private WPI_TalonFX shooterMotor2;
    private CANSparkMax ballLoader;
    private RelativeEncoder loaderEncoder;

    private boolean shooterMode = true; //True means high shooting, false means low shooting
    private BangBangController bangbangController;
    private SimpleMotorFeedforward feedforwardController;

    public Shooter() {
        
        shooterMotor1 = new WPI_TalonFX(Constants.SubsystemConstants.SHOOTER_MOTOR_1);
        shooterMotor2 = new WPI_TalonFX(Constants.SubsystemConstants.SHOOTER_MOTOR_2);
        ballLoader = new CANSparkMax(Constants.SubsystemConstants.BALL_LOADER, MotorType.kBrushless);
        bangbangController = new BangBangController();
        //feedforwardController = new SimpleMotorFeedforward(Constants.SubsystemConstants.kS, Constants.SubsystemConstants.kV, Constants.SubsystemConstants.kA);
        shooterMotor2.setInverted(true);
        shooterMotor2.follow(shooterMotor1);
        shooterMode = true;
        shooterMotor1.config_kP(Constants.SubsystemConstants.slotIdx, Constants.SubsystemConstants.kP);
        shooterMotor1.config_kI(Constants.SubsystemConstants.slotIdx, Constants.SubsystemConstants.kI);
        shooterMotor1.config_kD(Constants.SubsystemConstants.slotIdx, Constants.SubsystemConstants.kD);
        shooterMotor1.config_kF(Constants.SubsystemConstants.slotIdx, 0.063070283/1.32);
        

        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);
        ballLoader.setIdleMode(IdleMode.kBrake);
        loaderEncoder = ballLoader.getEncoder();
        shooterShuffleboard();
    }

    public void load(String mode) {
        if(mode.toUpperCase().equals("LOAD")){
            ballLoader.set(-Constants.SubsystemConstants.LOADER_SPEED);
        } else if(mode.toUpperCase().equals("UNLOAD")){
            ballLoader.set(Constants.SubsystemConstants.LOADER_SPEED);
        } else if (mode.toUpperCase().equals("STOP")) {
            ballLoader.set(0);
        
        } else { //Call this variable when sending a string
            ballLoader.set(Double.parseDouble(mode));
        }
    }

    public void run(String mode) {
        if(mode.toUpperCase().equals("START")){
            if(shooterMode){
                shooterMotor1.set(Constants.SubsystemConstants.shootHighSpeed);//*bangbangController.calculate(getLeftVelocity(),-Constants.SubsystemConstants.shootHighSpeed)); //+ Constants.SubsystemConstants.feedFordwardConstant * feedforwardController.calculate(Constants.SubsystemConstants.shootHighSpeed));
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
                //Set shooter speed based off BangBangController and FeedFordwardController (Calibrated with SysID)
            }
            else{
                shooterMotor1.set(Constants.SubsystemConstants.shootLowSpeed);//*bangbangController.calculate(getLeftVelocity(),-Constants.SubsystemConstants.shootLowSpeed)); //+ Constants.SubsystemConstants.feedFordwardConstant * feedforwardController.calculate(Constants.SubsystemConstants.shootLowSpeed));
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
            }
        } else if (mode.toUpperCase().equals("STOP")) {
            shooterMotor1.set(0);
        }    
        else {
            shooterMotor1.set(Double.parseDouble(mode));
        }
    }

    public void incrementShooterSpeed(){
        if(shooterMode) {
            Constants.SubsystemConstants.shootHighSpeed += 0.01;
            Constants.SubsystemConstants.shootHighVelocity += 250;
        }
        else {
            Constants.SubsystemConstants.shootLowSpeed += 0.01;
            Constants.SubsystemConstants.shootLowVelocity += 250;
        }
    }
    
    public void decrementShooterSpeed(){
        if(shooterMode) {
            Constants.SubsystemConstants.shootHighSpeed -= 0.01;
            Constants.SubsystemConstants.shootHighVelocity -= 250;
        }
        else {
            Constants.SubsystemConstants.shootLowSpeed -= 0.01;
            Constants.SubsystemConstants.shootLowVelocity -= 250;
        }
    }
    public double distanceToSpeed(double distance){
        //-4.50E5 is A, -0.02349 is B, 1.622E4 is C, 123.3 is a correction factor.
        //double speed = -4.50E5*Math.pow(10,distance*-0.02349)+1.622E4+123.3; //Exponential Fit
        double speed = Math.pow(810.5,distance*-0.01310)+7788.2; //Exponential Fit

        return speed;
    }
    
    public void runSmart(String mode) {
        if(mode.toUpperCase().equals("START")){
            if(shooterMode){
                shooterMotor1.set(TalonFXControlMode.Velocity, Constants.SubsystemConstants.shootHighVelocity);
                //System.out.println(0.75*feedforwardController.calculate(distanceToSpeed(95)));
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
                //Set shooter speed based off BangBangController and FeedFordwardController (Calibrated with SysID)
            }
            else{
                shooterMotor1.set(TalonFXControlMode.Velocity, Constants.SubsystemConstants.shootLowVelocity);
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
            }
        } else if (mode.toUpperCase().equals("STOP")) {
            shooterMotor1.set(0);
        }    
        else {
            shooterMotor1.set(Double.parseDouble(mode));
        }
    }
    public void runAuto(String mode, double velocity) {
        if(mode.toUpperCase().equals("START")){
                shooterMotor1.set(TalonFXControlMode.Velocity, velocity);
                //System.out.println(0.75*feedforwardController.calculate(distanceToSpeed(95)));
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
                //Set shooter speed based off BangBangController and FeedFordwardController (Calibrated with SysID)
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
            }
        
        else {
            shooterMotor1.set(0);
        }    
    }

    public void changeMode(){
        shooterMode = !shooterMode;
    }

    public boolean getMode(){
        return shooterMode;
    }
    
    public double getLeftVelocity() {
        return shooterMotor1.getSensorCollection().getIntegratedSensorVelocity();
    }

    public double getRightVelocity() { 
        return shooterMotor2.getSensorCollection().getIntegratedSensorVelocity();
        
    }

    public double getLoaderVelocity(){
        return loaderEncoder.getVelocity();
    }
    
    public void setShooterNeutralMode(NeutralMode mode){
        shooterMotor1.setNeutralMode(mode);
    }

    public void setLoaderIdleMode(IdleMode mode){
        ballLoader.setIdleMode(mode);
    }

    public void reportShooter(){
        //Graph config
        
        if(!Constants.SubsystemConstants.shuffleboardCompMode[5]){
            shooterLeftSpeedGraph.setDouble(getRightVelocity());
            shooterRightSpeedGraph.setDouble(getLeftVelocity());
        
        //Testing config
            shooterLeftSpeedEntry.setDouble(getLeftVelocity());
            shooterRightSpeedEntry.setDouble(getRightVelocity());
            ballLoaderSpeedEntry.setDouble(getLoaderVelocity());
        }
        else{
             //Compeitition config
            shooterModeComp.setBoolean(getMode());
            shooterHighSpeedComp.setDouble(Constants.SubsystemConstants.shootHighVelocity);
            shooterLowSpeedComp.setDouble(Constants.SubsystemConstants.shootLowVelocity);
            shooterLeftSpeedEntry.setDouble(getLeftVelocity());
            shooterRightSpeedEntry.setDouble(getRightVelocity());
    }
        }

       

    public void shooterShuffleboard(){
 
        if(!Constants.SubsystemConstants.shuffleboardCompMode[5]){
                   //Graph config
            shooterLeftSpeedGraph = graphTab.add("Left Shooter Velocity", getLeftVelocity())
                .withWidget(BuiltInWidgets.kGraph).getEntry();
            shooterRightSpeedGraph = graphTab.add("Right Shooter Velocity", getRightVelocity())
                .withWidget(BuiltInWidgets.kGraph).getEntry();
            
            //Testing config
            shooterLeftSpeedEntry = competitionTab.add("Left Shooter Velocity", getLeftVelocity()).getEntry();
            shooterRightSpeedEntry = competitionTab.add("Right Shooter Velocity", getRightVelocity()).getEntry();
            ballLoaderSpeedEntry = testingTab.add("Ball Loader Velocity", getLoaderVelocity()).getEntry();

            //Compeitiion Config
            shooterModeComp = competitionTab.add("Shoot High", getMode()).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            shooterHighSpeedComp = competitionTab.add("Current High Speed", Constants.SubsystemConstants.shootHighVelocity).getEntry(); 
            shooterLowSpeedComp = competitionTab.add("Current Low Speed", Constants.SubsystemConstants.shootLowVelocity).getEntry();
        }
        else{
            shooterModeComp = competitionTab.add("Shoot High", getMode()).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
            shooterHighSpeedComp = competitionTab.add("Current High Speed", Constants.SubsystemConstants.shootHighVelocity).getEntry(); 
            shooterLowSpeedComp = competitionTab.add("Current Low Speed", Constants.SubsystemConstants.shootLowVelocity).getEntry();
            shooterLeftSpeedEntry = competitionTab.add("Left Shooter Velocity", getLeftVelocity()).getEntry();
            shooterRightSpeedEntry = competitionTab.add("Right Shooter Velocity", getRightVelocity()).getEntry();
        }
    }

    @Override
    public void periodic(){
        //System.out.println("feed"+(feedforwardController.calculate(getRightVelocity())));
       // System.out.println("velocity"+getRightVelocity());
        reportShooter();
        //System.out.println("wingoblingo"+feedforwardController.calculate(0, 500, 1.5));
    }

}