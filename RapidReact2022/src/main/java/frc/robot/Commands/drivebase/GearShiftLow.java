package frc.robot.Commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;

public class GearShiftLow extends CommandBase{
     /*** Variables ***/


  private final DriveBase drivebase;
  boolean endCommand;

  public GearShiftLow(DriveBase m_drivebase) {
    drivebase = m_drivebase;
    
    addRequirements(drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    drivebase.shiftHighToLow();
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

