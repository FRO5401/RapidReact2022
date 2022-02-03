package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    
    private WPI_TalonSRX transMotor1;
    private WPI_TalonSRX transMotor2;
    private WPI_TalonSRX rotateMotor1;
    private WPI_TalonSRX rotateMotor2;
    private DigitalInput limit1;
    private DigitalInput limit2; 
    
    public Climber() {
        transMotor1 = new WPI_TalonSRX(Constants.SubsystemConstants.TRANS_MOTOR_1);
        transMotor2 = new WPI_TalonSRX(Constants.SubsystemConstants.TRANS_MOTOR_2);
        rotateMotor1 = new WPI_TalonSRX(Constants.SubsystemConstants.ROTATE_MOTOR_1);
        rotateMotor2 = new WPI_TalonSRX(Constants.SubsystemConstants.ROTATE_MOTOR_2);
        transMotor2.setInverted(true);
        rotateMotor2.setInverted(true);
        transMotor1.setNeutralMode(NeutralMode.Brake);
        transMotor2.setNeutralMode(NeutralMode.Brake);
        rotateMotor1.setNeutralMode(NeutralMode.Brake);
        rotateMotor2.setNeutralMode(NeutralMode.Brake);
    }

    public void setMotorSpeeds(String type, double speed){
        if(type.toUpperCase().contains("TRANS")){
            transMotor2.set(speed);
            transMotor1.set(speed);
        } else if (type.toUpperCase().contains("ROT")) {
            rotateMotor2.set(speed);
            rotateMotor1.set(speed);
        }
    }
    
    public double getMotorSpeeds(String type, int number){
        double returnable = 0;
        if(type.toUpperCase().contains("TRANS")){
            if(number == 1){
                returnable = transMotor1.getSelectedSensorVelocity();
            } else if (number == 2){
                returnable = transMotor2.getSelectedSensorVelocity();
            }
        } else if (type.toUpperCase().contains("ROT")) {
            if(number == 1){
                returnable = rotateMotor1.getSelectedSensorVelocity();
            } else if (number == 2){
                returnable = rotateMotor2.getSelectedSensorVelocity();
            }
        }
        return returnable;
    }

    @Override
    public void periodic() {
        reportClimber();
    }

    public void reportClimber(){
        SmartDashboard.putNumber("Translation Motor 1 Speed", getMotorSpeeds("TRANSLATION", 1));
        SmartDashboard.putNumber("Translation Motor 2 Speed", getMotorSpeeds("TRANSLATION", 2));
        SmartDashboard.putNumber("Rotation Motor 1 Speed", getMotorSpeeds("ROTATION", 1));
        SmartDashboard.putNumber("Rotation Motor 2 Speed", getMotorSpeeds("ROTATION", 2));
    }
}
