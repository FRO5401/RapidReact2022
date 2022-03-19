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
        mechEncoder2 = mechMotor2.getEncoder();
        mechEncoder = mechMotor.getEncoder();

        mechMotor.follow(mechMotor2, true);
        mechMotor.setSmartCurrentLimit(95);
        //100A is supposedly the sall current so we don't kill motor again
        setMechNeutralMode(IdleMode.kBrake);
        internalMechShuffleboard();
    }

    public void run(String mode) {
        if(mode.toUpperCase().equals("PULL"))
            mechMotor2.set(Constants.SubsystemConstants.MECH_SPEED);
        else if(mode.toUpperCase().equals("PUSH"))
            mechMotor2.set(-Constants.SubsystemConstants.MECH_SPEED);
        else if (mode.toUpperCase().equals("STOP"))
            mechMotor2.set(0);
        else  //Call this variable when sending a string
            mechMotor2.set(Double.parseDouble(mode));
    }
    
    //Set Motor Neutral
    public void setMechNeutralMode(IdleMode mode){
        mechMotor2.setIdleMode(mode);
    }

    public double getAverageIMVelocity(){
        return (mechEncoder2.getVelocity() + mechEncoder2.getVelocity())/2;
    }
    public double getLeftIMVelocity(){
        return (mechEncoder2.getVelocity());
    }
    public double getRightIMVelocity(){
        return (mechEncoder2.getVelocity());
    }

    //Reports Internal Mech Motor to Smart Dashboard

    public void reportSensors(){
        internalMechAverageGraph.setDouble(getAverageIMVelocity());
        internalMechAverageEntry.setDouble(getAverageIMVelocity());
        leftIMSpeedGraph.setDouble(getLeftIMVelocity());
        rightIMSpeedGraph.setDouble(getRightIMVelocity());
        leftIMSpeedEntry.setDouble(getLeftIMVelocity());
        rightIMSpeedEntry.setDouble(getRightIMVelocity());
    }

    //Internal Mech Shuffleboard
    public void internalMechShuffleboard(){
        //All Tabs
        internalMechAverageGraph = graphTab.add("Average Internal Mech Speed", getAverageIMVelocity()).withWidget(BuiltInWidgets.kGraph).getEntry(); 
        internalMechAverageEntry = testingTab.add("Average Internal Mech Speed", getAverageIMVelocity()).getEntry(); 
        leftIMSpeedGraph = graphTab.add("IM Left Motor Speed",getLeftIMVelocity())
            .withWidget(BuiltInWidgets.kGraph).getEntry();
        rightIMSpeedGraph = graphTab.add("IM Right Motor Speed",getRightIMVelocity())
        .withWidget(BuiltInWidgets.kGraph).getEntry(); 
        leftIMSpeedEntry = testingTab.add("Left Internal Mech Speed", getLeftIMVelocity()).getEntry(); 
        rightIMSpeedEntry = testingTab.add("Right Internal Mech Speed", getRightIMVelocity()).getEntry(); 


    }

    //Runs methods periodically
    @Override
    public void periodic() {
        reportSensors();
    }



    
}