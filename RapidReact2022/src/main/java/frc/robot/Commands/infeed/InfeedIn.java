package frc.robot.Commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Infeed;

public class InfeedIn extends CommandBase{
     /*** Variables ***/
  private final Infeed infeed;
  boolean endCommand=false;

  public InfeedIn(Infeed m_infeed) {
    infeed = m_infeed;
    
    addRequirements(infeed);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    infeed.run("INFEED");
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

