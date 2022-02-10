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

    public void run(String mode) {
        if(mode.toUpperCase().equals("PULL"))
            mechMotor.set(Constants.SubsystemConstants.MECH_SPEED);
        else if(mode.toUpperCase().equals("PUSH"))
            mechMotor.set(-Constants.SubsystemConstants.MECH_SPEED);
        else if (mode.toUpperCase().equals("STOP"))
            mechMotor.set(0);
        else  //Call this variable when sending a string
            mechMotor.set(Double.parseDouble(mode));
    }
    
    //Set Motor Neutral
    public void setMechNeutralMode(){
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