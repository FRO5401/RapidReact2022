package frc.robot.Commands.internal_mech;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.InternalMech;

public class BeltComplement extends CommandBase{
     /*** Variables ***/

  InternalMech internalMech;   
  String mode;

  public BeltComplement(InternalMech m_internalMech, String mode) {
    internalMech = m_internalMech;
    this.mode = mode;

    addRequirements(internalMech);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    internalMech.run(mode);
  }

  @Override
  public void end(boolean interrupted) {
    internalMech.run("STOP");
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