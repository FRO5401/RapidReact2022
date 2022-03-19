package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.actions.AutoBallInfeed;
import frc.robot.Autonomous.actions.AutoInfeedVisionDrive;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.NetworkTables;

public class BallCenterTest extends SequentialCommandGroup {

  boolean doneCommand;
  DriveBase drivebase;
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