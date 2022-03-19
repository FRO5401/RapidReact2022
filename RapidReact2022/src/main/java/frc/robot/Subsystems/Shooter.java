package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
        feedforwardController = new SimpleMotorFeedforward(Constants.SubsystemConstants.kS, Constants.SubsystemConstants.kV, Constants.SubsystemConstants.kA);
        shooterMotor2.setInverted(true);
        shooterMotor2.follow(shooterMotor1);
        shooterMode = true;
        

        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);
        ballLoader.setIdleMode(IdleMode.kBrake);
        loaderEncoder = ballLoader.getAlternateEncoder(Type.kQuadrature, 4096);
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
            Constants.SubsystemConstants.shootHighSpeed += 0.05;
        }
        else {
            Constants.SubsystemConstants.shootLowSpeed += 0.05;
        }
    }
    
    public void decrementShooterSpeed(){
        if(shooterMode) {
            Constants.SubsystemConstants.shootHighSpeed -= 0.05;
        }
        else {
            Constants.SubsystemConstants.shootLowSpeed -= 0.05;
        }
    }
    public double distanceToSpeed(double distance){
        //double speed = (1.768 *Math.pow(10, -5) * Math.pow(distance, 2)) + -0.001591*distance + 0.7997; //Quadratic Fit
        double speed = 0.002617*distance + 0.5584; //Linear Fit
        return speed;
    }
    
    public void runSmart(String mode) {
        if(mode.toUpperCase().equals("START")){
            if(shooterMode){
                shooterMotor1.set(feedforwardController.calculate(15000));
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
                //Set shooter speed based off BangBangController and FeedFordwardController (Calibrated with SysID)
            }
            else{
                shooterMotor1.set(feedforwardController.calculate(15000));
                //shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
            }
        } else if (mode.toUpperCase().equals("STOP")) {
            shooterMotor1.set(0);
        }    
        else {
            shooterMotor1.set(Double.parseDouble(mode));
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
        shooterLeftSpeedGraph.setDouble(getRightVelocity());
        shooterRightSpeedGraph.setDouble(getLeftVelocity());
        
        //Testing config
        shooterLeftSpeedEntry.setDouble(getLeftVelocity());
        shooterRightSpeedEntry.setDouble(getRightVelocity());
        ballLoaderSpeedEntry.setDouble(getLoaderVelocity());

        //Compeitition config
        shooterModeComp.setBoolean(getMode());
        shooterHighSpeedComp.setDouble(Constants.SubsystemConstants.shootHighSpeed);
        shooterLowSpeedComp.setDouble(Constants.SubsystemConstants.shootLowSpeed);
    }

    public void shooterShuffleboard(){
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
        shooterHighSpeedComp = competitionTab.add("Current High Speed", Constants.SubsystemConstants.shootHighSpeed).getEntry(); 
        shooterLowSpeedComp = competitionTab.add("Current Low Speed", Constants.SubsystemConstants.shootLowSpeed).getEntry();
    }

    @Override
    public void periodic(){
        //System.out.println("feed"+(feedforwardController.calculate(getRightVelocity())));
       // System.out.println("velocity"+getRightVelocity());
        reportShooter();
    }

}