package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static frc.robot.Tabs.*;

public class Infeed extends SubsystemBase {
    //things go here
    private Solenoid infeedGate;
    boolean deploy = true;
    
   // private TalonSRX infeedMotor1;
    //private TalonSRX infeedMotor2;

    private CANSparkMax infeedMotor1;
    private CANSparkMax infeedMotor2;
    private RelativeEncoder im1Encoder;
    private RelativeEncoder im2Encoder;

    public Infeed() {
        infeedGate = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SubsystemConstants.INFEED_GATE);
        infeedMotor1 = new CANSparkMax(Constants.SubsystemConstants.INFEED_SPARK_1, MotorType.kBrushless);
        infeedMotor2 = new CANSparkMax(Constants.SubsystemConstants.INFEED_SPARK_2, MotorType.kBrushless);
        //infeedMotor1 = new TalonSRX(Constants.SubsystemConstants.INFEED_SPARK_1);
        //infeedMotor2 = new TalonSRX(Constants.SubsystemConstants.INFEED_SPARK_2); 
        infeedMotor2.follow(infeedMotor1, true);
       // infeedMotor2.setInverted(true);  
       /// infeedMotor2.follow(infeedMotor1);  
        infeedMotor1.setSmartCurrentLimit(40, 1);
        infeedMotor2.setSmartCurrentLimit(40, 1);
        infeedMotor1.burnFlash();
        

        im1Encoder = infeedMotor1.getEncoder();
        im2Encoder = infeedMotor2.getEncoder();
        
        setInfeedIdleMode(IdleMode.kBrake);
        toggleGate();
        infeedShuffleboard();
        
    }

    public void toggleGate() {
        deploy = !deploy;
        infeedGate.set(deploy);
    }

    public boolean getGateState(){
        return infeedGate.get();
    }

    public void run(String mode) {
        if(mode.toUpperCase().contains("IN")) {
            infeedMotor1.set(Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
        } else if(mode.toUpperCase().contains("OUT")) {
            infeedMotor1.set(-Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
        } else if (mode.toUpperCase().equals("STOP")) {
            infeedMotor1.set(0);
        }
        else { //Call this variable when sending a strings
            infeedMotor1.set(Double.parseDouble(mode));
        }
        System.out.println("testInfeed");
    }

    public void setInfeedIdleMode(IdleMode mode){
        infeedMotor1.setIdleMode(mode);
    }


    public void reportInfeed() {
        //Graph reporting
        infeedLeftSpeedGraph.setDouble(getLeftInfeedSpeed());
        infeedRightSpeedGraph.setDouble(getRightInfeedSpeed());
        
        //Testing reporting
        infeedLeftSpeedEntry.setDouble(getLeftInfeedSpeed());
        infeedRightSpeedEntry.setDouble(getRightInfeedSpeed());
        gateEntry.setBoolean(getGateState());

        //Compeition reporting
        gateComp.setBoolean(getGateState());
    }

    public double getLeftInfeedSpeed(){
       return im1Encoder.getVelocity();
       //return infeedMotor1.getSensorCollection().getQuadratureVelocity();
    }

    public double getRightInfeedSpeed(){
       return im2Encoder.getVelocity();
       //return infeedMotor1.getSensorCollection().getQuadratureVelocity();
    }

    public void infeedShuffleboard(){
        //Graph config
        infeedLeftSpeedGraph = graphTab.add("Left Infeed Motor Graph",getLeftInfeedSpeed())
            .withWidget(BuiltInWidgets.kGraph).getEntry();
        infeedRightSpeedGraph = graphTab.add("Right Infeed Motor Graph",getRightInfeedSpeed())
            .withWidget(BuiltInWidgets.kGraph).getEntry(); 

  
        //Testing Tab
        gateEntry = testingTab.add("Infeed Gate", getGateState()).getEntry();
        infeedLeftSpeedEntry = testingTab.add("Left Infeed Motor Speed",getLeftInfeedSpeed()).getEntry();
        infeedRightSpeedEntry = testingTab.add("Right Infeed Motor Speed",getRightInfeedSpeed()).getEntry(); 

        //Comp Tab
        gateComp = testingTab.add("Infeed Gate State", getGateState())
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    }

    @Override
    public void periodic() {
        reportInfeed();        
    }
}
