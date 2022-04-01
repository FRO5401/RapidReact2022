package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Autonomous.actions.AutoPIDDrive;
import frc.robot.Subsystems.DriveBase;

public class DriveStraight extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public DriveStraight(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    addCommands(new AutoPIDDrive(DistanceInput, SpeedInput, passedDrivebase));
  }
}