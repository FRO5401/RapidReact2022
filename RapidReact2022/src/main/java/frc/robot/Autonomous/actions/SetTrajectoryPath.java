package frc.robot.Autonomous.actions;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveBase;

public class SetTrajectoryPath extends CommandBase {
    private DriveBase drivebase;
    private String trajectoryJSON;
    private Trajectory exampleTrajectory = new Trajectory();
    private boolean doneTravelling = false;

	public SetTrajectoryPath(DriveBase passedDrivebase, String trajectoryJSONPath) {
        trajectoryJSON = trajectoryJSONPath;
        drivebase = passedDrivebase;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
           } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            System.out.println("Error");
          }
    }      

	// Called just before this Command runs the first time
    @Override
    public void initialize() {

    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        RamseteCommand ramseteCommand =
          new RamseteCommand(
              exampleTrajectory,
              drivebase::getPose,
              new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
              new SimpleMotorFeedforward(
                  Constants.AutoConstants.ksVolts,
                  Constants.AutoConstants.kvVoltSecondsPerMeter,
                  Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
              Constants.AutoConstants.kDriveKinematics,
              drivebase::getWheelSpeeds,
              new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
              new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
              // RamseteCommand passes volts to the callback
              drivebase::tankDriveVolts,
              drivebase);
  
        // Reset odometry to the starting pose of the trajectory.
        drivebase.resetOdometry(exampleTrajectory.getInitialPose());
        ramseteCommand.andThen(() -> drivebase.tankDriveVolts(0, 0));
        doneTravelling = true;
    }

    // Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		drivebase.tankDriveVolts(0,0);
	}

	// Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return doneTravelling;
    }
}