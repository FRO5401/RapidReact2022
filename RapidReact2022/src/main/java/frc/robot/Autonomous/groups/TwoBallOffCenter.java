package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Autonomous.actions.AutoTurn;
import frc.robot.Commands.drivebase.ResetSensors;
import frc.robot.Commands.infeed.InfeedIn;
import frc.robot.Commands.infeed.InfeedStop;
import frc.robot.Commands.internal_mech.StartBelt;
import frc.robot.Commands.internal_mech.StopBelt;
import frc.robot.Commands.shooter.IncrementShooter;
import frc.robot.Commands.shooter.ShootBall;
import frc.robot.Commands.shooter.StartLoad;
import frc.robot.Commands.shooter.StopLoad;
import frc.robot.Commands.shooter.StopShooter;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.Shooter;

public class TwoBallOffCenter extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public TwoBallOffCenter(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Shooter passedShooter, InternalMech passedInternalMech, Climber passedClimber, Infeed passedInfeed) {
    addCommands(
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ParallelCommandGroup(
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new ShootBall(passedShooter)
      ),
      //new AutoTurn(0.3, passedDrivebase.getGyroAngle(), passedDrivebase),
      new WaitCommand(2.5),
      new StartBelt(passedInternalMech),
      new StartLoad(passedShooter),
      new WaitCommand(3),
      //new StopShooter(passedShooter),
      new StopBelt(passedInternalMech),
      new StopLoad(passedShooter),
      new AutoTurn(0.3, -90, passedDrivebase),
      new ParallelCommandGroup(
        new InfeedIn(passedInfeed),
        new StartLoad(passedShooter)
      ),
      new AutoDrive(80, 0.3, passedDrivebase),
      new WaitCommand(1.5),
      new InfeedStop(passedInfeed),
      new StopLoad(passedShooter),
      new IncrementShooter(passedShooter),
      new AutoTurn(0.3, 115, passedDrivebase),
      new WaitCommand(1.5),
      new StartBelt(passedInternalMech),
      new StartLoad(passedShooter),
      new WaitCommand(2),
      new StopBelt(passedInternalMech),
      new StopLoad(passedShooter),
      new StopShooter(passedShooter)
    );    
  }
}