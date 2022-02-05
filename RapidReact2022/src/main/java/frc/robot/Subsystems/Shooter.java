package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Shooter extends SubsystemBase{
 
    private WPI_TalonFX shooterMotor1;
    private WPI_TalonFX shooterMotor2;
    private double velocity;
    public Shooter() {
        
        shooterMotor1 = new WPI_TalonFX(Constants.DriveConstants.SHOOTER_MOTOR_1);
        shooterMotor2 = new WPI_TalonFX(Constants.DriveConstants.SHOOTER_MOTOR_2);
        

        shooterMotor2.follow(shooterMotor1);
        
        shooterMotor2.setInverted(true);
        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);
    }
    public void init() {
        shooterMotor1.set(TalonFXControlMode.PercentOutput, 0.0); //for aesthetics
    }
    public void stop() {
        shooterMotor1.set(TalonFXControlMode.PercentOutput, 0.0); //for more aesthetics
    }

    public void runMotors(double speed){
        shooterMotor1.set(TalonFXControlMode.PercentOutput, speed);
    }
    
    public double getVelocity() {
        velocity =  shooterMotor1.getSensorCollection().getIntegratedSensorVelocity();

        return velocity;
    }
    public void setShooterNeutralMode(NeutralMode mode){
        shooterMotor1.setNeutralMode(mode);
    }
    public void periodic(){
        SmartDashboard.putNumber("ShooterVelocity", velocity);
    }

}
