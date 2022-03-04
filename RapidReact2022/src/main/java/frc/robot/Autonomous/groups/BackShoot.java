package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.actions.AutoDrive;
import frc.robot.Autonomous.actions.AutoTurn;
import frc.robot.Commands.drivebase.ResetSensors;
import frc.robot.Commands.internal_mech.BeltComplement;
import frc.robot.Commands.internal_mech.StartBelt;
import frc.robot.Commands.internal_mech.StopBelt;
import frc.robot.Commands.shooter.ShootBall;
import frc.robot.Commands.shooter.StopShooter;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.Shooter;

public class BackShoot extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public BackShoot(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Shooter passedShooter, InternalMech passedInternalMech) {
    addCommands(
      new ResetSensors(passedDrivebase),
      new ResetSensors(passedDrivebase),
      new ResetSensors(passedDrivebase),
      new ResetSensors(passedDrivebase),
      new ResetSensors(passedDrivebase),
      new ParallelCommandGroup(
        new AutoDrive(DistanceInput, SpeedInput, passedDrivebase),
        new ShootBall(passedShooter)
      ),
      //new AutoTurn(0.3, -passedDrivebase.getGyroAngle(), passedDrivebase),
      new WaitCommand(2),
      new StartBelt(passedInternalMech),
      new WaitCommand(3),
      new StopShooter(passedShooter),
      new StopBelt(passedInternalMech)
    );    
  }

  /*@Override
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
    }*/
}