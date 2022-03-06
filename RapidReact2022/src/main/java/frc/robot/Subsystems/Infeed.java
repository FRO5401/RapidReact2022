package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static frc.robot.Tabs.*;

public class Infeed extends SubsystemBase {
    //things go here
    private Solenoid infeedGate;
    boolean deploy = true;
    
    private VictorSPX infeedMotor1;
    private VictorSPX infeedMotor2;

    public Infeed() {
        infeedGate = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SubsystemConstants.INFEED_GATE);
        infeedMotor1 = new VictorSPX(Constants.SubsystemConstants.INFEED_SPARK_1);
        infeedMotor2 = new VictorSPX(Constants.SubsystemConstants.INFEED_SPARK_2);
        infeedMotor2.setInverted(true);
        infeedMotor2.follow(infeedMotor1);
        
        setInfeedNeutralMode(NeutralMode.Brake);
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
        System.out.println("STATUS2");
        if(mode.toUpperCase().contains("IN")) {
            infeedMotor1.set(VictorSPXControlMode.PercentOutput, Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
        } else if(mode.toUpperCase().contains("OUT")) {
            infeedMotor1.set(VictorSPXControlMode.PercentOutput, -Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
        } else if (mode.toUpperCase().equals("STOP")) {
            infeedMotor1.set(VictorSPXControlMode.PercentOutput, 0);
        }
        else { //Call this variable when sending a strings
            infeedMotor1.set(VictorSPXControlMode.PercentOutput, Double.parseDouble(mode));
        }
    }

    public void setInfeedNeutralMode(NeutralMode mode){
        infeedMotor1.setNeutralMode(mode);
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
        return infeedMotor1.getSelectedSensorVelocity();
    }

    public double getRightInfeedSpeed(){
        return infeedMotor2.getSelectedSensorVelocity();
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
