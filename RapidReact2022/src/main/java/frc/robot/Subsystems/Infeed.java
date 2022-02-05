package frc.robot.Subsystems;

//All of these imports are just here till I figure out which ones we need - David
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Controls;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



public class Infeed extends SubsystemBase {
    //things go here
    private Solenoid gate;
    
    private CANSparkMax infeedMotor1;
    private CANSparkMax infeedMotor2;
    private RelativeEncoder iM1Encoder;
    private RelativeEncoder iM2Encoder;

    public Infeed() {
        gate = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SubsystemConstants.INFEED_GATE);
        infeedMotor1 = new CANSparkMax(Constants.SubsystemConstants.INFEED_SPARK_1, MotorType.kBrushless);
        infeedMotor2 = new CANSparkMax(Constants.SubsystemConstants.INFEED_SPARK_2, MotorType.kBrushless);
        infeedMotor2.setInverted(true);
        
        //These are the encoders that are on the motors, 4096 is the Counts Per Rev.
        iM1Encoder = infeedMotor1.getAlternateEncoder(Type.kQuadrature, 4096);
        iM2Encoder = infeedMotor1.getAlternateEncoder(Type.kQuadrature, 4096);

    }

    public void gateDrop() {
        //this will drop gate
        gate.set(true);
    }

    public void setMotors(double speed) {
        //This will set the speed of both motors, keep in mind that one motor is inverted.
        infeedMotor1.set(speed);
        infeedMotor2.set(speed);
    }

    public void infeedIn() {
        setMotors(Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
    }

    public void infeedOut() {
        setMotors(Constants.SubsystemConstants.INFEED_MOTOR_SPEED*-1);
    }

    public void infeedIdleMode(IdleMode mode){
        infeedMotor1.setIdleMode(mode);
        infeedMotor2.setIdleMode(mode);
    }

    public void infeedStop() {
        setMotors(0);
    }

    public void reportInfeed() {
        SmartDashboard.putBoolean("Gate Solenoid", gate.get());
        SmartDashboard.putNumber("Infeed Motor 1 Velocity", iM1Encoder.getVelocity());
        SmartDashboard.putNumber("Infeed Motor 2 Velocity", iM2Encoder.getVelocity());
        SmartDashboard.putNumber("Infeed Motor 1 Input", infeedMotor1.get());
        SmartDashboard.putNumber("Infeed Motor 2 Input", infeedMotor2.get());
    }

    @Override
    public void periodic() {
        reportInfeed();        
    }
}
