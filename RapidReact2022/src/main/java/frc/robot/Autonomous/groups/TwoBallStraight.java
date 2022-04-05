package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Autonomous.actions.AutoTurn;
import frc.robot.Commands.drivebase.ResetSensors;
import frc.robot.Commands.infeed.GateToggle;
import frc.robot.Commands.infeed.InfeedIn;
import frc.robot.Commands.infeed.InfeedStop;
import frc.robot.Commands.internal_mech.StartBelt;
import frc.robot.Commands.internal_mech.StopBelt;
import frc.robot.Commands.shooter.DecrementShooter;
import frc.robot.Commands.shooter.IncrementShooter;
import frc.robot.Commands.shooter.StartShooter;
import frc.robot.Commands.shooter.StartLoad;
import frc.robot.Commands.shooter.StopLoad;
import frc.robot.Commands.shooter.StopShooter;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.Shooter;

public class TwoBallStraight extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public TwoBallStraight(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Shooter passedShooter, InternalMech passedInternalMech, Climber passedClimber, Infeed passedInfeed) {
    addCommands(
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new ResetSensors(passedDrivebase, passedClimber),
      new GateToggle(passedInfeed),
      new ParallelCommandGroup(
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new InfeedIn(passedInfeed)
      ),
      //new AutoTurn(0.3, passedDrivebase.getGyroAngle(), passedDrivebase),
      new WaitCommand(0.5), //technically unnecessary
      new StartBelt(passedInternalMech),
      new WaitCommand(0.5), 
      new ParallelCommandGroup(
        new StopBelt(passedInternalMech),
        new AutoTurn(0.7, 180, passedDrivebase),
        new InfeedStop(passedInfeed),
        new StartShooter(passedShooter)
      ),
      new WaitCommand(1.5),
      new StartBelt(passedInternalMech),
      new StartLoad(passedShooter),
      new WaitCommand(3),
      new StopBelt(passedInternalMech),
      new StopLoad(passedShooter),
      new StopShooter(passedShooter)
    );    
  }
}