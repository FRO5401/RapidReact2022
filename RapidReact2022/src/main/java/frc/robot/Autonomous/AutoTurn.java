package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveBase;

public class AutoTurn extends CommandBase {

    private DriveBase drivebase;
	private double desiredAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;

	public AutoTurn(double SpeedInput, double AngleInput, DriveBase passedDrivebase) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);
        drivebase = passedDrivebase;
		desiredAngle = AngleInput;
		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
        addRequirements(drivebase);
	}

	// Called just before this Command runs the first time
    @Override
    public void initialize() {

        //drivebase.resetSensors();
        //drivebase.setDPPHighGear();
        drivebase.setDPPLowGear();
        drivebase.resetGyroAngle();
        doneTraveling = false;

    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        drivebase.autoTurn(autoDriveSpeed, desiredAngle);
        if(drivebase.getGyroAngle() > desiredAngle+2 || drivebase.getGyroAngle() < desiredAngle-2)
            doneTraveling = true;
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
}