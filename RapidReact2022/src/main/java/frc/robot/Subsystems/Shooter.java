package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Shooter extends SubsystemBase{
 
    private WPI_TalonFX shooterMotor1;
    private WPI_TalonFX shooterMotor2;
    private WPI_TalonSRX ballLoader;
    private boolean shooterMode = true; //True means high shooting, false means low shooting

    public Shooter() {
        
        shooterMotor1 = new WPI_TalonFX(Constants.SubsystemConstants.SHOOTER_MOTOR_1);
        shooterMotor2 = new WPI_TalonFX(Constants.SubsystemConstants.SHOOTER_MOTOR_2);
        ballLoader = new WPI_TalonSRX(Constants.SubsystemConstants.BALL_LOADER);

        shooterMotor2.follow(shooterMotor1);
        
        shooterMotor2.setInverted(true);
        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);
    }

    public void load(String mode) {
        if(mode.toUpperCase().equals("LOAD"))
            ballLoader.set(-Constants.SubsystemConstants.LOADER_SPEED);
        else if(mode.toUpperCase().equals("UNLOAD"))
            ballLoader.set(Constants.SubsystemConstants.LOADER_SPEED);    
        else if (mode.toUpperCase().equals("STOP")) 
            ballLoader.set(0);   
        else //Call this variable when sending a string
            ballLoader.set(Double.parseDouble(mode));
    }

    public void run(String mode) {
        if(mode.toUpperCase().equals("START")){
            ballLoader.set(Constants.SubsystemConstants.SHOOTER_SPEED);
        } else if(mode.toUpperCase().equals("STOP")){
            ballLoader.set(0);
        } else { //Call this variable when sending a string
            ballLoader.set(Double.parseDouble(mode));
        }
    }
    
    public void changeMode(){
        shooterMode = !shooterMode;
    }
    
    public double getVelocity() {
        return shooterMotor1.getSensorCollection().getIntegratedSensorVelocity();
    }
    
    public void setShooterNeutralMode(NeutralMode mode){
        shooterMotor1.setNeutralMode(mode);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    }

}