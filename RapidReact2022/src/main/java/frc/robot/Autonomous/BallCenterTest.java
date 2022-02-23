package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.NetworkTables;

public class BallCenterTest extends SequentialCommandGroup {

  boolean doneCommand;
  DriveBase drivebase;
  /**
   * Add your docs here.
   */
  public BallCenterTest(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables) {
    drivebase = passedDrivebase;
    addCommands(
        new AutoBallInfeed(SpeedInput, passedDrivebase, passedNetworkTables),
        new WaitCommand(1),
        new AutoVisionDrive(SpeedInput, passedDrivebase, passedNetworkTables)
    );
    
  }
}