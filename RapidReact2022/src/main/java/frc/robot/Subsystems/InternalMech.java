package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import static frc.robot.Tabs.*;

public class InternalMech extends SubsystemBase{ 
    private CANSparkMax mechMotor; 
    private RelativeEncoder mechEncoder;
    
    public InternalMech(){
        mechMotor = new CANSparkMax(Constants.SubsystemConstants.INTERNAL_MECH_MOTOR, MotorType.kBrushless);
        mechEncoder = mechMotor.getAlternateEncoder(Type.kQuadrature, 4096);
        mechMotor.setInverted(true);
        internalMechShuffleboard();
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
    public void setMechIdleMode(IdleMode mode){
        mechMotor.setIdleMode(mode);
    }

    public double getVelocity(){
        return mechEncoder.getVelocity();
    }

    //Reports Internal Mech Motor to Smart Dashboard
    public void reportSensors(){
        internalMechGraph.setDouble(getVelocity());
        internalMechEntry.setDouble(getVelocity());
    }

    //Internal Mech Shuffleboard
    public void internalMechShuffleboard(){
        //All Tabs
        internalMechGraph = graphTab.add("Internal Mech Speed", getVelocity()).withWidget(BuiltInWidgets.kGraph).getEntry(); 
        internalMechEntry = testingTab.add("Internal Mech Speed", getVelocity()).getEntry(); 
    }

    //Runs methods periodically
    @Override
    public void periodic() {
        reportSensors();
    }



    
}