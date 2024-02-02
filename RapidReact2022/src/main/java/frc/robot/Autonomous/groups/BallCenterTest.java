package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.actions.AutoBallInfeed;
import frc.robot.Autonomous.actions.AutoInfeedVisionDrive;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;

public class BallCenterTest extends SequentialCommandGroup {

  boolean doneCommand;
  private DriveBase drivebase;
	private NetworkTables networktables;
	private Shooter shooter;
  private Infeed infeed;
  private Climber climber;
  private InternalMech internalMech;
  /**
   * Add your docs here.
   */
  public BallCenterTest(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables, Infeed passedInfeed) {
    addCommands(
        new AutoBallInfeed(SpeedInput, passedDrivebase, passedNetworkTables, passedInfeed),
        new WaitCommand(1),
        new AutoInfeedVisionDrive(SpeedInput, passedDrivebase, passedNetworkTables, passedInfeed)
    );
    
  }
/** 
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0,0);
  }

  @Override
  public boolean isFinished(){
    return doneCommand;
  }
*/
}