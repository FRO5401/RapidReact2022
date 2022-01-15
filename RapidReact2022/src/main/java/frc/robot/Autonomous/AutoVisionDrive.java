
package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoVisionDrive extends Command {

    //private int ballCount;
	private double angle;
	private double desiredDistance;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;

	public AutoVisionDrive(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
        distanceTraveled = 0;
        //ballCount = 0;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		Robot.drivebase.resetSensors();
		Robot.drivebase.setDPPHighGear();
		Robot.drivebase.setDPPLowGear();

		doneTraveling = false;
		distanceTraveled = 0;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        //Robot.infeed.runMotors();
        desiredDistance = (Robot.networktables.getBallDistance() - 5);

		if ((distanceTraveled) <= (desiredDistance)) {
			Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
			doneTraveling = false;
		} else if (distanceTraveled >= (desiredDistance)) {
            Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
            doneTraveling = false;
		} else {
            //if(Robot.drummag.getBallCount() > ballCount){
			    Robot.drivebase.stopMotors();
                doneTraveling = true;
            //}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return doneTraveling;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivebase.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.drivebase.stopMotors();
	}

}
