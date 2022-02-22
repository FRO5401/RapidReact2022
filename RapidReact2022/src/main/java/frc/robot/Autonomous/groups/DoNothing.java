package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Subsystems.DriveBase;

public class DoNothing extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public DoNothing(DriveBase passedDrivebase) {
    addCommands(new AutoDrive(0, 0, passedDrivebase));
  }
}