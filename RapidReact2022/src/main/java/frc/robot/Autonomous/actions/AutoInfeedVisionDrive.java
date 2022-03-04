
package frc.robot.Autonomous.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.NetworkTables;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoInfeedVisionDrive extends CommandBase {

    //private int ballCount;
	private double angle;
	private double desiredDistance;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;
	private DriveBase drivebase;
	private NetworkTables networktables;

	public AutoInfeedVisionDrive(double SpeedInput, DriveBase passedDriveBase, NetworkTables passedNetworkTables) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

		drivebase = passedDriveBase;
		networktables = passedNetworkTables;
		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
        distanceTraveled = 0;
        //ballCount = 0;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {

		drivebase.resetEncoders();
		drivebase.resetGyroAngle();
		drivebase.DPPShifter("HIGH");
		drivebase.DPPShifter("LOW");

		doneTraveling = false;
		distanceTraveled = 0;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
        //Robot.infeed.runMotors();
		distanceTraveled = drivebase.getPosition();
		angle = drivebase.getGyroYaw();
        desiredDistance = (networktables.getBallDistance())*59/43;

		if ((distanceTraveled) <= (desiredDistance)) {
			drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
			doneTraveling = false;
		} else if (distanceTraveled >= (desiredDistance)) {
			//made negatove
            drivebase.autoDrive(-autoDriveSpeed, -autoDriveSpeed, angle);
            doneTraveling = false;
		} else {
            //if(Robot.drummag.getBallCount() > ballCount){
			    drivebase.drive(0,0);
                doneTraveling = true;
            //}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return doneTraveling;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		drivebase.drive(0,0);
	}

  	@Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}

}
