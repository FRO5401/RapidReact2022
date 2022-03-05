package frc.robot.Commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;
import frc.robot.Utilities.testers.Printer;


import static frc.robot.Controls.*;
/*
 * Command controls the following drive functions:
 * - TURNING
 * - FORWARDS/BACKWARDS
 * - TURN IN PLACE
 * - GEAR SHIFT HIGH/LOW
 */

public class RotateClimberArms extends CommandBase {
  /*** Variables ***/
    //Input Axes
  double rotation;
  double left;
  boolean checkRotation;
  boolean checkOverExtension;
  double right; 
  double sensitivity;

  private final Climber climber;

  public RotateClimberArms(Climber m_climber) {
    climber = m_climber;
    
    
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
   
    Printer.print("Climber-Rot");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    rotation = xboxAxis(operator, "LS-X").getAxis();
    checkRotation = xboxAxis(operator, "LS-X").get();
    if(checkRotation && !climber.checkOverExtension(climber.getLeftRotAngle()))
      climber.setMotorSpeeds("ROT", rotation);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotorSpeeds("ROT",0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
    public boolean runsWhenDisabled() {
      return false;
  }
}