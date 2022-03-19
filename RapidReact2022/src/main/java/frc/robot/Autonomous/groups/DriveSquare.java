package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Autonomous.actions.AutoTurn;
import frc.robot.Subsystems.DriveBase;

public class DriveSquare extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public DriveSquare(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    addCommands(
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.3, 90, passedDrivebase),
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.3, 90, passedDrivebase),
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.3, 90, passedDrivebase),
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.3, 90, passedDrivebase)
    );
  }

}