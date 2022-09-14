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
import frc.robot.Commands.infeed.StartInfeed;
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
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;
import frc.robot.Constants;


public class TwoBallStraight extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
	private NetworkTables networktables;
	private Shooter shooter;
  private Infeed infeed;
  private Climber climber;
  private InternalMech internalMech;

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
        new StartInfeed(passedInfeed)
      ),
      //new AutoTurn(0.3, passedDrivebase.getGyroAngle(), passedDrivebase),
      new WaitCommand(0.5), //technically unnecessary
      new StartBelt(passedInternalMech),
      new WaitCommand(0.5), 
      new IncrementShooter(passedShooter),
      new ParallelCommandGroup(
        new StopBelt(passedInternalMech),
        new AutoTurn(0.5, 175, passedDrivebase),
        new InfeedStop(passedInfeed),
        new StartShooter(passedShooter)
      ),
      new WaitCommand(1.5),
      new StartBelt(passedInternalMech),
      new StartLoad(passedShooter),
      new WaitCommand(3),
      new StopBelt(passedInternalMech),
      new StopLoad(passedShooter),
      new StopShooter(passedShooter),
      new DecrementShooter(passedShooter),

      //onto 3 ball
      
      new WaitCommand(0.5),
      new AutoTurn(0.6, -70, passedDrivebase),
      new GateToggle(passedInfeed),
      
      new WaitCommand(.5),
       new ParallelCommandGroup(
        new AutoDrive(/*Trying to find but direct cm does not work that well, closet we got was using the value of 150 * */ Constants.AutoConstants.SCUFFED_CORRECTION_CONSTANT, SpeedInput, passedDrivebase),
        new StartInfeed(passedInfeed)
      ),
      
      new WaitCommand(0.5)//, //technically unnecessary
      //new AutoDrive(-10 * Constants.AutoConstants.SCUFFED_CORRECTION_CONSTANT, SpeedInput, passedDrivebase) //, //back up so don't slam wall
      /*
      new StartBelt(passedInternalMech),
      new WaitCommand(0.5), 
      new IncrementShooter(passedShooter),
      new ParallelCommandGroup(
        new StopBelt(passedInternalMech),
        new AutoTurn(0.5, z, passedDrivebase),
        new InfeedStop(passedInfeed),
        new StartShooter(passedShooter)
      ),
      new DecrementShooter(passedShooter)
      */
    );    
  }
}