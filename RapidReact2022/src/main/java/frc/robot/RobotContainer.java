package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.groups.BallCenterTest;
import frc.robot.Autonomous.groups.DoNothing;
import frc.robot.Autonomous.groups.DriveStraight;
import frc.robot.Commands.drivebase.*;
import frc.robot.Commands.infeed.*;
import frc.robot.Commands.internal_mech.*;
import frc.robot.Commands.shooter.*;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.controllers.MultipleInputGroup;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import static frc.robot.Controls.*;
import static frc.robot.Tabs.*;

public class RobotContainer {
    
    private final SendableChooser<Command> chooser = new SendableChooser<Command>();
    // The robot's subsystems
    private final DriveBase drivebase = new DriveBase();
    private final NetworkTables networktables= new NetworkTables();
    private final Infeed infeed = new Infeed();
    //private final InternalMech internalMech = new InternalMech();
    private final Shooter shooter = new Shooter();

    private final MultipleInputGroup drivetrain = new MultipleInputGroup();

    public RobotContainer() {
        configureInputGroups();
        configureButtonBindings();
        chooser.setDefaultOption("Do Nothing", new DoNothing(drivebase));
        chooser.addOption("Drive Straight", new DriveStraight(200, 0.3, drivebase));
       // chooser.addOption("Ball Center Test", new BallCenterTest(0.3, drivebase, networktables));
        //chooser.addOption("Trajectory Test", new SetTrajectoryPath(drivebase, "paths/DriveStraight.wpilib.json")); //REPLACE LATER
        SmartDashboard.putData("Auto choices", chooser);
        
    }

    public boolean updateDrivetrain(){
        return drivetrain.get();
    }

    public DriveBase getDriveBase(){
        return drivebase;
    }

    private void configureCommandShuffleboard() {

    }

    private void configureInputGroups(){
        drivetrain.addAxis(xboxLT_Driver);
        drivetrain.addAxis(xboxRT_Driver);
        drivetrain.addAxis(xboxLX_Driver);
        drivetrain.addButton(xboxRightBumper_Driver);
        drivetrain.addButton(xboxLeftBumper_Driver);
        drivetrain.addButton(xboxL3_Driver);
    }

    private void configureButtonBindings() {

        //Sets the default command of drivebase to an array of things needed to drive normally
        drivetrain.whenAnyActive(new XboxMove(drivebase));

        //Drivebase Controls      
        xboxButton(operator, "Back").whenPressed(new ResetSensors(drivebase));
        xboxButton(driver, "Start").whenPressed(new GearShiftHigh(drivebase));
        xboxButton(driver, "Back").whenPressed(new GearShiftLow(drivebase));
        xboxButton(driver, "Y").whenPressed(new CompressorToggle(drivebase));

        //Subsystem Controls
        //infeed
        xboxButton(operator, "RB").whenHeld(new ParallelRaceGroup(
            new InfeedIn(infeed)//, 
           // new BeltComplement(internalMech, "PULL"),
            //new LoadBall(shooter, "LOAD")
            ));
        xboxButton(operator, "LB").whenHeld(new ParallelRaceGroup(
            new InfeedOut(infeed)//,
           // new BeltComplement(internalMech, "PUSH"),
            //new LoadBall(shooter, "UNLOAD")
            ));
        xboxButton(operator, "B").whenPressed(new GateToggle(infeed));

        //internal mechanism
       //// xboxDPad(operator, 0).whenHeld(new BeltComplement(internalMech, "PULL"));
       // xboxDPad(operator, 180).whenHeld(new BeltComplement(internalMech, "PUSH"));
        
        //shooter
       xboxButton(operator, "A").whenHeld(new SequentialCommandGroup(
            new ShootBall(shooter),
            new WaitCommand(Constants.SubsystemConstants.SHOOTER_WAIT_TIME), //Guessed wait time
            new ParallelRaceGroup(
            //    new BeltComplement(internalMech, "PULL"),
                new LoadBall(shooter, "UNLOAD")
            ))).whenReleased(new StopShooter(shooter));
        xboxButton(operator, "Y").whenPressed(new ChangeMode(shooter));
        //TODO: Change this stuff back before I forget

    }

    public Command getAutonomousCommand(){
    // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              Constants.AutoConstants.ksVolts,
              Constants.AutoConstants.kvVoltSecondsPerMeter,
              Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
              Constants.AutoConstants.kMaxSpeedMetersPerSecond,
              Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.AutoConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    /*Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config);*/

  

    // Run path following command, then stop at the end.
    return chooser.getSelected();
    }

}
