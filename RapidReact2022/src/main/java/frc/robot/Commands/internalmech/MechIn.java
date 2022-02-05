package frc.robot.Commands.internalmech;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.InternalMech;

public class MechIn extends CommandBase{
     /*** Variables ***/

  boolean endCommand;

  public MechIn() {

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    endCommand = true;
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return endCommand;
  }

  @Override
    public boolean runsWhenDisabled() {
      return false;
  }
}

