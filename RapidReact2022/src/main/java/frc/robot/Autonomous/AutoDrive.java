package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveBase;

public class AutoDrive extends CommandBase {

    private DriveBase drivebase;
	private double angle;
	private double desiredDistance;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;

	public AutoDrive(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);

        drivebase = passedDrivebase;
		desiredDistance = DistanceInput;
		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
        distanceTraveled = 0;
        addRequirements(drivebase);
	}

	// Called just before this Command runs the first time
    @Override
    public void initialize() {

        //drivebase.resetSensors();
        drivebase.setDPPHighGear();
        drivebase.setDPPLowGear();

        doneTraveling = false;
        distanceTraveled = 0;

    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        angle = drivebase.getGyroAngle();
        distanceTraveled = drivebase.getEncoderDistance(2) * Constants.DriveConstants.LOW_GEAR_RIGHT_DPP;
        if ((distanceTraveled) <= (desiredDistance) && desiredDistance >= 0) {
            drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
            doneTraveling = false;
        } else if (distanceTraveled >= (desiredDistance) && desiredDistance < 0) {
            drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
        } else {
            drivebase.drive(0,0);
            doneTraveling = true;
        }
    }

    // Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		drivebase.drive(0,0);
	}

	// Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return doneTraveling;
    }

    @Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}
}
