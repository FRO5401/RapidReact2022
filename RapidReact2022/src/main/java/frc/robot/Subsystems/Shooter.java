package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        feedforwardController = new SimpleMotorFeedforward(Constants.SubsystemConstants.kS, Constants.SubsystemConstants.kS, Constants.SubsystemConstants.kS);

        shooterMotor2.follow(shooterMotor1);
        
        shooterMotor2.setInverted(true);
        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);
        ballLoader.setIdleMode(IdleMode.kBrake);
    }

    public void load(String mode) {
        if(mode.toUpperCase().equals("LOAD")){
            ballLoader.set(-Constants.SubsystemConstants.LOADER_SPEED);
        } else if(mode.toUpperCase().equals("UNLOAD")){
            ballLoader.set(Constants.SubsystemConstants.LOADER_SPEED);
        } else { //Call this variable when sending a string
            ballLoader.set(Double.parseDouble(mode));
        }
    }

    public void run(String mode) {
        if(mode.toUpperCase().equals("START")){
            ballLoader.set(Constants.SubsystemConstants.SHOOTER_SPEED);
        } else if(mode.toUpperCase().equals("STOP")){
            ballLoader.set(0);
        } else { //Call this variable when sending a string
            ballLoader.set(Double.parseDouble(mode));
        }

        if(shooterMode){
            shooterMotor1.set(bangbangController.calculate(getVelocity(),Constants.SubsystemConstants.shootHighSpeed) 
            + Constants.SubsystemConstants.feedFordwardConstant * feedforwardController.calculate(Constants.SubsystemConstants.shootHighSpeed));
            //Set shooter speed based off BangBangController and FeedFordwardController (Calibrated with SysID)
        }
        else{
            shooterMotor1.set(bangbangController.calculate(getVelocity(),Constants.SubsystemConstants.shootLowSpeed) 
            + Constants.SubsystemConstants.feedFordwardConstant * feedforwardController.calculate(Constants.SubsystemConstants.shootLowSpeed));
        }
    }
    
    public void changeMode(){
        shooterMode = !shooterMode;
    }
    
    public double getVelocity() {
        return shooterMotor1.getSensorCollection().getIntegratedSensorVelocity();
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

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
        SmartDashboard.putNumber("Ball Loader Velocity", getLoaderVelocity());
    }

}