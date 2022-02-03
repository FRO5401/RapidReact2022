package frc.robot.Commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;

public class ResetSensors extends CommandBase{
     /*** Variables ***/

     //Testing Buttons (TODO: Remove for Comp)
  boolean resetSensors;

  private final DriveBase drivebase;

  public ResetSensors(DriveBase m_drivebase) {
    drivebase = m_drivebase;
    
    addRequirements(drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    drivebase.shiftHighToLow();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
      drivebase.resetEncoders();
      drivebase.resetGyroAngle();
      drivebase.resetTalon(true);
      drivebase.resetTalon(false); // ;)
      System.out.println("reset sensors");
    
  }

  @Override
  public void end(boolean interrupted) {
    
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
