package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Autonomous.*;
import frc.robot.Commands.drivebase.*;
import frc.robot.Subsystems.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {
    
    private final SendableChooser<Command> chooser = new SendableChooser<Command>();
    // The robot's subsystems
    public final DriveBase drivebase = new DriveBase();
    public final NetworkTables networktables= new NetworkTables();
    public final Climber climber = new Climber();
    //private final CompressorSubsystem compressor = new CompressorSubsystem();


    public RobotContainer() {
        
        configureButtonBindings();
        chooser.setDefaultOption("Do Nothing", new DoNothing(drivebase));
        chooser.addOption("Drive Straight", new DriveStraight(200, 0.3, drivebase));
        chooser.addOption("Ball Center Test", new BallCenterTest(0.3, drivebase, networktables));
        //chooser.addOption("Trajectory Test", new SetTrajectoryPath(drivebase, "paths/DriveStraight.wpilib.json")); //REPLACE LATER
        SmartDashboard.putData("Auto choices", chooser);
    }

    private void configureButtonBindings() {

        //Sets the default command of drivebase to an array of things needed to drive normally
        drivebase.setDefaultCommand(
            new XboxMove(
                drivebase,
                () -> Controls.xboxAxis(Controls.driver, "LT"),
                () -> Controls.xboxAxis(Controls.driver, "RT"),
                () -> Controls.xboxAxis(Controls.driver, "LS-X"),
                () -> Controls.xboxButton(Controls.driver, "LS").get(),
                () -> Controls.xboxButton(Controls.driver, "RB").get(),
                () -> Controls.xboxButton(Controls.driver, "LB").get()));

        //driver and operator controls for drivebase        
        Controls.xboxButton(Controls.operator, "Back").whenPressed(new ResetSensors(drivebase));
        Controls.xboxButton(Controls.driver, "Start").whenPressed(new GearShiftHigh(drivebase));
        Controls.xboxButton(Controls.driver, "Back").whenPressed(new GearShiftLow(drivebase));

        //driver and operator controls for subsystems
        //Controls.xboxButton(Controls.operator, "Start").whenPressed(new ClimberRoutine(climber));
        //Controls.xboxButton(Controls.operator, "X").whenPressed(new StopClimber(climber));
        

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
