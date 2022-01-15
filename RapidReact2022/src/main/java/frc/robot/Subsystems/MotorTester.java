package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


/**
 * Add your docs here.
 */
public class MotorTester extends SubsystemBase {


  // Configuring Motors
  private WPI_TalonSRX smartMotor; 
  private WPI_VictorSPX regularMotor1;
  private WPI_VictorSPX regularMotor2;


  

  public MotorTester() {

    smartMotor = new WPI_TalonSRX(RobotMap.SMART_MOTOR); 
    regularMotor1 = new WPI_VictorSPX(RobotMap.REGULAR_MOTOR_1);
    regularMotor2  = new WPI_VictorSPX(RobotMap.REGULAR_MOTOR_2);

    //smartMotor.setInverted(true);
    //regularMotor1.setInverted(true);
    //regularMotor2.setInverted(true);
    
  }

  @Override
  public void periodic() {
    reportMotors();
  }


  //Smart Motor is 1, RegularMotor1 is 2, RegularMotor2 is 3
  public void runMotor(double input, String mode, int motorNum) {
    var motor = (motorNum == 1) ? smartMotor : (motorNum == 2) ? regularMotor1 : regularMotor2;
    var control = (mode.toLowerCase().equals("percent") ? ControlMode.PercentOutput : (mode.toLowerCase().equals("velocity") ? ControlMode.Velocity : ControlMode.Position)); 
        motor.set(control, input);     
  }


  //Set voltage on motors
  public void setVolts(double input, int motorNum) {
    var motor = (motorNum == 1) ? smartMotor : (motorNum == 2) ? regularMotor1 : regularMotor2;
    motor.setVoltage(input);
  }

  public void reportMotors(){
    SmartDashboard.putNumber("Smart Motor Velocity", smartMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Smart Motor Position", smartMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Regular Motor 1 Velocity", regularMotor1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Regular Motor 1 Position", regularMotor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Regular Motor 2 Velocity", regularMotor2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Regular Motor 2 Position", regularMotor2.getSelectedSensorPosition());
  }
}