// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Move;

import com.revrobotics.CANSparkMaxLowLevel;

public class DriveTrain extends SubsystemBase {
  // creating canspark objects
  CANSparkMax lDrive1 =  new CANSparkMax(Constants.L1pid,CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax lDrive2 =  new CANSparkMax(Constants.L2pid,CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rDrive1 =  new CANSparkMax(Constants.R1pid,CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rDrive2 =  new CANSparkMax(Constants.R2pid,CANSparkMaxLowLevel.MotorType.kBrushless);
  // grouping said objects
  MotorControllerGroup leftControl = new MotorControllerGroup(lDrive1, lDrive2);
  MotorControllerGroup rightControl = new MotorControllerGroup(rDrive1, rDrive2);
  DifferentialDrive diffDrive = new DifferentialDrive(leftControl, rightControl);


  public DriveTrain() {
    //restoring just incase something goes horribly wrong
    lDrive1.restoreFactoryDefaults();
    lDrive2.restoreFactoryDefaults();
    rDrive1.restoreFactoryDefaults();
    rDrive2.restoreFactoryDefaults();
    // back follows front so you know, it actually moves
    lDrive2.follow(lDrive1);
    rDrive2.follow(rDrive1);

  }
  //inverting because on motor is backwards
  public void rightMotors(double speed){
    rDrive1.set(-speed);
    rDrive2.set(-speed);
  }
  //setting a speed
  public void leftMotors(double speed){
    lDrive1.set(speed);
    lDrive2.set(speed);
  }
// when the subsystem is ran itll run the Move command
  @Override
  public void periodic() {
    setDefaultCommand(new Move());
  }
}
