package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utilities.testers.Printer;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



public class Infeed extends SubsystemBase {
    //things go here
    //private Solenoid gateLeft;
    //private Solenoid gateRight;
    private Solenoid infeedGate;
    boolean deploy = false;
    
    private CANSparkMax infeedMotor1;
    private CANSparkMax infeedMotor2;

    public Infeed() {
        infeedGate = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SubsystemConstants.INFEED_GATE);
        //gateRight = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SubsystemConstants.INFEED_GATE_RIGHT);
        infeedMotor1 = new CANSparkMax(Constants.SubsystemConstants.INFEED_SPARK_1, MotorType.kBrushless);
        infeedMotor2 = new CANSparkMax(Constants.SubsystemConstants.INFEED_SPARK_2, MotorType.kBrushless);
        infeedMotor2.follow(infeedMotor1, true);

        setInfeedIdleMode(IdleMode.kBrake);
        
    }

    public void toggleGate() {
        deploy = !deploy;
        System.out.println("deploys");
        //gateLeft.set(deploy);
        //gateRight.set(deploy);
        infeedGate.set(deploy);
    }

    public void run(String mode) {
        if(mode.toUpperCase().contains("IN")) {
            infeedMotor1.set(Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
        } else if(mode.toUpperCase().contains("OUT")) {
            Printer.qp(9832);
            infeedMotor1.set(-Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
        } else if (mode.toUpperCase().equals("STOP")) {
            infeedMotor1.set(0);
        }
        else { //Call this variable when sending a string
            infeedMotor1.set(Double.parseDouble(mode));
        }
    }

    public void setInfeedIdleMode(IdleMode mode){
        infeedMotor1.setIdleMode(mode);
    }


    public void reportInfeed() {
        //SmartDashboard.putBoolean("Left Gate Solenoid", gateLeft.get());
        //SmartDashboard.putBoolean("Right Gate Solenoid", gateRight.get());
        SmartDashboard.putBoolean("Both gates", infeedGate.get());
        SmartDashboard.putNumber("Infeed Motor 1 Input", infeedMotor1.get());
        SmartDashboard.putNumber("Infeed Motor 2 Input", infeedMotor2.get());
    }

    @Override
    public void periodic() {
        reportInfeed();        
    }
}
