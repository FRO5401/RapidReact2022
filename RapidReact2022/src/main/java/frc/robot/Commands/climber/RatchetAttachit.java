package frc.robot.Commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;
import frc.robot.Utilities.testers.Printer;

public class RatchetAttachit extends CommandBase{
     /*** Variables ***/

  boolean resetSensors;

  private final Climber climber;
  boolean endCommand=false;

  public RatchetAttachit(Climber m_climber) {
    climber = m_climber;
    
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Printer.print("Rachet");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    climber.setSafetySolenoid();
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
      return true;
  }
}

