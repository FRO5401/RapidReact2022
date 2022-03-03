package frc.robot.Commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Utilities.testers.Printer;

public class ResetSensors extends CommandBase{
     /*** Variables ***/

     boolean endCommand = false;

  private final DriveBase drivebase;

  public ResetSensors(DriveBase m_drivebase) {
    drivebase = m_drivebase;
    
    addRequirements(drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    drivebase.shift("LOW");
    Printer.print("ResetSensors");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
      drivebase.resetEncoders();
      drivebase.resetGyroAngle();
      System.out.println("RESET SENSORS");
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
