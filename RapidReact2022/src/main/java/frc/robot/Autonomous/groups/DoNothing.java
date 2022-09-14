package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;

public class DoNothing extends SequentialCommandGroup {
  
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
  public DoNothing(DriveBase passedDrivebase) {
    drivebase = passedDrivebase;
    addCommands(new AutoDrive(0, 0.0, passedDrivebase));

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
