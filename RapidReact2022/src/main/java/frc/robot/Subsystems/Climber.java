package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    
    //Creates parts for the robot
    private CANSparkMax transMotor1;
    private CANSparkMax transMotor2;
    private CANSparkMax rotateMotor1;
    private CANSparkMax rotateMotor2;
    private RelativeEncoder tMEncoder1;
    private RelativeEncoder tMEncoder2;
    private RelativeEncoder rMEncoder1;
    private RelativeEncoder rMEncoder2;
    private DigitalInput limit1;
    private DigitalInput limit2; 
    
    public Climber() {
        //Instantiates the motors and limits
        transMotor1 = new CANSparkMax(Constants.SubsystemConstants.TRANS_MOTOR_1, MotorType.kBrushless);
        transMotor2 = new CANSparkMax(Constants.SubsystemConstants.TRANS_MOTOR_2, MotorType.kBrushless);
        rotateMotor1 = new CANSparkMax(Constants.SubsystemConstants.ROTATE_MOTOR_1, MotorType.kBrushless);
        rotateMotor2 = new CANSparkMax(Constants.SubsystemConstants.ROTATE_MOTOR_2, MotorType.kBrushless);
        tMEncoder1 = transMotor1.getAlternateEncoder(4096);
        tMEncoder2 = transMotor2.getAlternateEncoder(4096);
        rMEncoder1 = rotateMotor1.getAlternateEncoder(4096);
        rMEncoder2 = rotateMotor2.getAlternateEncoder(4096);
        limit1 = new DigitalInput(Constants.SubsystemConstants.DIGITAL_INPUT_1);
        limit2 = new DigitalInput(Constants.SubsystemConstants.DIGITAL_INPUT_2);

        //Inverts the necessary motors
        transMotor2.setInverted(true);
        rotateMotor2.setInverted(true);

        //Makes sure the neutral mode is on brake
        setClimberIdleMode("Translation", IdleMode.kBrake);
        setClimberIdleMode("Climber", IdleMode.kBrake);
    }

    //Set motor speeds for the climber
    public void setMotorSpeeds(String type, double speed){
        if(type.toUpperCase().contains("TRANS")){
            transMotor2.set(speed);
            transMotor1.set(speed);
        } else if (type.toUpperCase().contains("ROT")) {
            rotateMotor2.set(speed);
            rotateMotor1.set(speed);
        }
    }
    
    //Get motor speeds of the two different climber motors
    public double getMotorSpeeds(String type, int number){
        double returnable = 0;
        if(type.toUpperCase().contains("TRANS")){
            if(number == 1){
                returnable = tMEncoder1.getVelocity();
            } else if (number == 2){
                returnable = tMEncoder2.getVelocity();
            }
        } else if (type.toUpperCase().contains("ROT")) {
            if(number == 1){
                returnable = rMEncoder1.getVelocity();
            } else if (number == 2){
                returnable = rMEncoder2.getVelocity();
            }
        }
        return returnable;
    }

    public boolean getLimit1(){
        return limit1.get();
    }

    public boolean getLimit2(){
        return limit2.get();
    }

    public void setClimberIdleMode(String type, IdleMode mode){
        if(type.toUpperCase().contains("TRANS")){
            transMotor1.setIdleMode(mode);
            transMotor2.setIdleMode(mode);
        } else if (type.toUpperCase().contains("ROT")) {
            rotateMotor1.setIdleMode(mode);
            rotateMotor2.setIdleMode(mode);
        }
    }

    //Reports the climber sensors periodically
    @Override
    public void periodic() {
        reportClimber();
    }

    //Reports the speeds of the climber
    public void reportClimber(){
        SmartDashboard.putNumber("Translation Motor 1 Speed", getMotorSpeeds("TRANSLATION", 1));
        SmartDashboard.putNumber("Translation Motor 2 Speed", getMotorSpeeds("TRANSLATION", 2));
        SmartDashboard.putNumber("Rotation Motor 1 Speed", getMotorSpeeds("ROTATION", 1));
        SmartDashboard.putNumber("Rotation Motor 2 Speed", getMotorSpeeds("ROTATION", 2));
        SmartDashboard.putBoolean("Limit 1 status", getLimit1());
        SmartDashboard.putBoolean("Limit 2 status", getLimit2());
    }
}
