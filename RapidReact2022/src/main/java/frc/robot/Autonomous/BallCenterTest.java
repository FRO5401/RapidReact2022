package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.NetworkTables;

public class BallCenterTest extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public BallCenterTest(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables) {
    addCommands(
        new AutoBallInfeed(SpeedInput, passedDrivebase, passedNetworkTables),
        new WaitCommand(1),
        new AutoVisionDrive(SpeedInput, passedDrivebase, passedNetworkTables)
    );
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}