package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    /**
     * Determines the speed at which the loading motors will go.
     * LOAD: Loading speed,
     * UNLOAD: Unloading speed,
     * CUSTOM: Set custom speed
     */
    public enum LoadMode { LOAD, UNLOAD}
    public enum RunMode { START, STOP}
 
    private WPI_TalonFX shooterMotor1;
    private WPI_TalonFX shooterMotor2;
    private WPI_TalonFX ballLoader;
    private boolean shooterMode = true; //True means high shooting, false means low shooting

    public Shooter() {
        
        shooterMotor1 = new WPI_TalonFX(Constants.SubsystemConstants.SHOOTER_MOTOR_1);
        shooterMotor2 = new WPI_TalonFX(Constants.SubsystemConstants.SHOOTER_MOTOR_2);
        shooterMotor2 = new WPI_TalonFX(Constants.SubsystemConstants.BALL_LOADER);

        shooterMotor2.follow(shooterMotor1);
        
        shooterMotor2.setInverted(true);
        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Changes the state of the load motor
     * @param mode Whether to set motor to load or unload mode
     */
    public void setLoaderState(LoadMode mode) {
        if(mode == LoadMode.LOAD){
            ballLoader.set(-Constants.SubsystemConstants.LOADER_SPEED);
        } else if(mode == LoadMode.UNLOAD){
            ballLoader.set(Constants.SubsystemConstants.LOADER_SPEED);
        }
    }
        /**
     * Changes the state of the load motor
     * @param speed Sets the motor to this speed
     */
    public void setLoaderState(double speed) {
        ballLoader.set(speed);
    }

    /**
     * Sets the state of the shooter
     * @param mode the mode can be Start or stop. Pretty self-explanatory
     */
    public void setState(RunMode mode) {
        if(mode == RunMode.START){
            shooterMotor1.set(Constants.SubsystemConstants.SHOOTER_SPEED);
        } else if(mode == RunMode.STOP){
            ballLoader.set(0);
        }
    }
    /**
     * Sets the state of the shooter
     * @param speed Sets the speed of the shooter
     */
    public void setState(double speed) {
        shooterMotor1.set(speed);
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