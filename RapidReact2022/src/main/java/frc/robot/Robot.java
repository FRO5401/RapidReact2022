// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;

public class Robot extends TimedRobot {
  private Command autoSelected;
  private RobotContainer robotContainer;

//  Robot() {
  //  super(0.05);
 // }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffleboard.update();
  }

  @Override
  public void autonomousInit() {
    autoSelected = robotContainer.getAutonomousCommand();

    if(autoSelected != null) {
      autoSelected.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if(autoSelected != null) {
      autoSelected.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if(!robotContainer.updateDrivetrain())
      robotContainer.getDriveBase().drive(0, 0);
    
  }
  
  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void testInit() {}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}
  
  

}