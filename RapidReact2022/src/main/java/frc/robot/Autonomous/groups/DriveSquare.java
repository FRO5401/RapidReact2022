package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Autonomous.actions.AutoTurn;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;

public class DriveSquare extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
	private NetworkTables networktables;
	private Shooter shooter;
  private Infeed infeed;
  private Climber climber;
  private InternalMech internalMech;

  public DriveSquare(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    addCommands(
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.5, 90, passedDrivebase),
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.5, 90, passedDrivebase),
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.5, 90, passedDrivebase),
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new AutoTurn(0.5, 90, passedDrivebase)
    );
  }

}