package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Autonomous.actions.AutoTurn;
import frc.robot.Commands.drivebase.ResetSensors;
import frc.robot.Commands.internal_mech.StartBelt;
import frc.robot.Commands.internal_mech.StopBelt;
import frc.robot.Commands.shooter.StartShooter;
import frc.robot.Commands.shooter.StartLoad;
import frc.robot.Commands.shooter.StopLoad;
import frc.robot.Commands.shooter.StopShooter;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;

public class BackShoot extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
	private NetworkTables networktables;
	private Shooter shooter;
  private Infeed infeed;
  private Climber climber;
  private InternalMech internalMech;

  public BackShoot(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Shooter passedShooter, InternalMech passedInternalMech, Climber passedClimber) {
    drivebase = passedDrivebase;
    internalMech = passedInternalMech;
    climber = passedClimber;
    shooter = passedShooter;

    addCommands(
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ParallelCommandGroup(
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new StartShooter(passedShooter)
      ),
      //new AutoTurn(0.3, passedDrivebase.getGyroAngle(), passedDrivebase),
      new WaitCommand(2.5),
      new StartBelt(passedInternalMech),
      new StartLoad(passedShooter),
      new WaitCommand(3),
      new StopShooter(passedShooter),
      new StopBelt(passedInternalMech),
      new StopLoad(passedShooter)
    );
    
    
    
  }

  @Override 
  public void end(boolean interrupted) {
    System.out.println("RUNS");
    shooter.run("STOP");
    internalMech.run("STOP");
    shooter.load("STOP");
  }
}