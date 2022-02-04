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
    
    private Spark motor1;
    private Spark motor2;

    public Infeed() {
        gate = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SubsystemConstants.INFEED_GATE);
        motor1 = new Spark(Constants.SubsystemConstants.INFEED_SPARK_1);
        motor2 = new Spark(Constants.SubsystemConstants.INFEED_SPARK_2);
        motor2.setInverted(true);

        //something else here maybe
    }

    public void gateDrop() {
        //this will drop gate
        gate.set(true);
    }

    public void setMotors(double speed) {
        //This will set the speed of both motors, keep in mind that one motor is inverted.
        motor1.set(speed);
        motor2.set(speed);
    }

    public void infeedIn() {
        setMotors(Constants.SubsystemConstants.INFEED_MOTOR_SPEED);
    }

    public void infeedOut() {
        setMotors(Constants.SubsystemConstants.INFEED_MOTOR_SPEED*-1);
    }

    public void infeedStop() {
        setMotors(0);
    }

    public void reportInfeed() {
        SmartDashboard.putBoolean("Gate Solenoid", gate.get());
        SmartDashboard.putNumber("Infeed Motor 1", motor1.get());
        SmartDashboard.putNumber("Infeed Motor 2", motor2.get());
    }

    @Override
    public void periodic() {
        reportInfeed();        
    }
}
