package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.*;
import frc.robot.Commands.drivebase.*;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.MultipleInputGroup;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {
    
    private final SendableChooser<Command> chooser = new SendableChooser<Command>();
    // The robot's subsystems
    private final DriveBase drivebase = new DriveBase();
    private final NetworkTables networktables= new NetworkTables();

    private final MultipleInputGroup drivetrain = new MultipleInputGroup();
    //private final CompressorSubsystem compressor = new CompressorSubsystem();


    public RobotContainer() {
        
        configureInputGroups();
        configureButtonBindings();
        chooser.setDefaultOption("Do Nothing", new DoNothing(drivebase));
        chooser.addOption("Drive Straight", new DriveStraight(100, 0.6, drivebase));
        chooser.addOption("Ball Center Test", new BallCenterTest(0.55, drivebase, networktables));
        //chooser.addOption("Trajectory Test", new SetTrajectoryPath(drivebase, "paths/DriveStraight.wpilib.json")); //REPLACE LATER
        Shuffleboard.getTab("SmartDashboard").add("Auto choices", chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    public boolean updateDrivetrain(){
        return drivetrain.get();
    }

    public DriveBase getDriveBase(){
        return drivebase;
    }

    private void configureButtonBindings() {

        //Drivebase Controls      
        Controls.xboxButton(Controls.operator, "Back").whenPressed(new ResetSensors(drivebase));
       // Controls.xboxButton(Controls.driver, "Start").whenPressed(new GearShiftHigh(drivebase));
        //Controls.xboxButton(Controls.driver, "Back").whenPressed(new GearShiftLow(drivebase));
        drivetrain.whenAnyActive(new XboxMove(drivebase));
    }

    private void configureInputGroups(){
        drivetrain.addAxis(Controls.xboxLT_Driver);
        drivetrain.addAxis(Controls.xboxRT_Driver);
        drivetrain.addAxis(Controls.xboxLX_Driver);
        drivetrain.addButton(Controls.xboxRightBumper_Driver);
        drivetrain.addButton(Controls.xboxLeftBumper_Driver);
        drivetrain.addButton(Controls.xboxL3_Driver);
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
