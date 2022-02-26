package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import static frc.robot.Tabs.*;

public class InternalMech extends SubsystemBase{ 
    private CANSparkMax mechMotor; 
    private CANSparkMax mechMotor2; 
    private RelativeEncoder mechEncoder;
    private RelativeEncoder mechEncoder2;
    
    public InternalMech(){
        // As of 2/25/22 assembly switched back to the Sparks. 2/26 we use two now.  -David
        mechMotor = new CANSparkMax(Constants.SubsystemConstants.INTERNAL_MECH_MOTOR, MotorType.kBrushless);
        mechMotor2 = new CANSparkMax(Constants.SubsystemConstants.INTERNAL_MECH_MOTOR2, MotorType.kBrushless);
        mechEncoder2 = mechMotor2.getAlternateEncoder(Type.kQuadrature, 4096);
        mechEncoder = mechMotor.getAlternateEncoder(Type.kQuadrature, 4096);

        mechMotor.follow(mechMotor2, true);
        mechMotor.setSmartCurrentLimit(95);
        //100A is supposedly the sall current so we don't kill motor again
        setMechNeutralMode(IdleMode.kBrake);
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
    public void setMechNeutralMode(IdleMode mode){
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