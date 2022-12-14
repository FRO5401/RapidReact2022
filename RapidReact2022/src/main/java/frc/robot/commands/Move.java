// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// wooo imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Move extends CommandBase {
  /** Creates a new Controller. */

  public Move() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //just letting you know that it runs fine
    System.out.println("Starting teleOP/controller");
  }

  // Called every time the scheduler runs while the command is scheduled.
  // setting speed based off axis
  @Override
  public void execute() {
    double LeftY = RobotContainer.container.rawAxis(Constants.leftY);
    double RightY = RobotContainer.container.rawAxis(Constants.rightY);
   Robot.driveTrain.leftMotors(LeftY);
   Robot.driveTrain.rightMotors(RightY);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
