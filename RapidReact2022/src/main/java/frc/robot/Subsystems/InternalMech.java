package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class InternalMech extends SubsystemBase{
    private WPI_TalonSRX mechMotor;
    
    public InternalMech(){
        mechMotor = new WPI_TalonSRX(Constants.SubsystemConstants.INTERNAL_MECH_MOTOR);

    }

    public void setMechSpeeds(double speed){
        mechMotor.set(speed);
    }

//Move Motor Forward
    public void mechPull() {
        setMechSpeeds(Constants.SubsystemConstants.MECH_SPEED);


    }
//Move Motor Backward
    public void mechPush() {
        setMechSpeeds(-Constants.SubsystemConstants.MECH_SPEED); 



    }
//Set Motor Neutral
    public void MechNeutralMode(){
        mechMotor.setNeutralMode(NeutralMode.Coast);

    }
//Reports Internal Mech Motor to Smart Dashboard
    public void reportSensors(){
        SmartDashboard.putNumber("Mech Speed", mechMotor.getSelectedSensorVelocity());
    }

//Runs methods periodically
    @Override
    public void periodic() {
        reportSensors();
    }

}
