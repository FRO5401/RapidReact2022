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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


/**
 * Add your docs here.
 */
public class SolenoidTester extends SubsystemBase {


  // Configuring Motors
  private DoubleSolenoid ds; 
  private Solenoid s1;
  private Solenoid s2;

  public SolenoidTester() {
    ds = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.DOUBLE_SOLENOID_1_PORT_1, RobotMap.DOUBLE_SOLENOID_1_PORT_2); 
    s1 = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_1);
    s2  = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.SOLENOID_2);
  }

  @Override
  public void periodic() {
    reportSolenoids();
  }

  //s1 is 1, s2 is 2
  public void activateSolenoids(boolean input, int solenoidNum) {
    var solenoid = (solenoidNum == 1) ? s1 : s2;
    solenoid.set(input);     
  }

  //s1 is 1, s2 is 2
  public void activateDoubleSolenoids(String mode, int solenoidNum) {
    var doubleSolenoid = ds;
    var control = (mode.toLowerCase().equals("forward") ? DoubleSolenoid.Value.kForward : (mode.toLowerCase().equals("reverse") ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kOff)); 
    doubleSolenoid.set(control);     
  }

  public void reportSolenoids(){
    SmartDashboard.putString("Double Solenoid 1 State", ds.get().toString());
    SmartDashboard.putBoolean("Solenoid 1 State", s1.get());
    SmartDashboard.putBoolean("Solenoid 2 State", s2.get());
  }
}