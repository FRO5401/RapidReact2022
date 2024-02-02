package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.actions.AutoBallShoot;
import frc.robot.Commands.internal_mech.StartBelt;
import frc.robot.Commands.shooter.StartLoad;
import frc.robot.Commands.shooter.StopLoad;
import frc.robot.Commands.shooter.StopShooter;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.InternalMech;
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;

public class BallShooterTest extends SequentialCommandGroup {
   
    private DriveBase drivebase;
	private NetworkTables networktables;
	private Shooter shooter;
    private Infeed infeed;
    private Climber climber;
    private InternalMech internalMech;
   
    public BallShooterTest(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables, Shooter passedShooter, InternalMech passedInternalMech) {
        addCommands(
            new AutoBallShoot(SpeedInput*.9, passedDrivebase, passedNetworkTables, passedShooter),
            new WaitCommand(2),
            new ParallelCommandGroup(
                new StartBelt(passedInternalMech),
                new StartLoad(passedShooter)
            ),
            new WaitCommand(3),
            new StopShooter(passedShooter),
            new StopLoad(passedShooter)
        );
    }
}