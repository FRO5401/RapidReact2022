package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
 
    WPI_TalonFX shooterMotor1;
    WPI_TalonFX shooterMotor2;

    public Shooter() {
        shooterMotor1 = new WPI_TalonFX(Constants.DriveConstants.SHOOTER_MOTOR_1);
        shooterMotor2 = new WPI_TalonFX(Constants.DriveConstants.SHOOTER_MOTOR_2);
        shooterMotor2.setInverted(true);
        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);
    }
}
