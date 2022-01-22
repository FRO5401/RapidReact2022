package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveBase;

public class DriveStraight extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  boolean doneCommand;
  DriveBase drivebase;

  public DriveStraight(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    addCommands(new AutoDrive(DistanceInput, SpeedInput, passedDrivebase));
    drivebase = passedDrivebase;
    doneCommand = true;

  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0,0);
  }

  @Override
  public boolean isFinished(){
    return doneCommand;
  }

  @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}