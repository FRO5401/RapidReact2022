package frc.robot.Commands.internal_mech;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.InternalMech;

public class BeltComplement extends CommandBase{
     /*** Variables ***/

  InternalMech internalMech;   
  boolean endCommand;
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