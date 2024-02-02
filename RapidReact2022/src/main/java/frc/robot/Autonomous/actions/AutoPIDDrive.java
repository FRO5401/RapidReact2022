package frc.robot.Autonomous.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;

public class AutoPIDDrive extends CommandBase {

    private DriveBase drivebase;
	private double angle, desiredDistance, autoDriveSpeed, distanceTraveled; //Can declare variables next to each other
	private boolean doneTraveling;

	public AutoPIDDrive(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);
        doneTraveling = false;
        drivebase = passedDrivebase;
		desiredDistance = DistanceInput;
		autoDriveSpeed = SpeedInput;
        distanceTraveled = 0;
        addRequirements(drivebase);
	}

	// Called just before this Command runs the first time
    @Override
    public void initialize() {

        drivebase.resetEncoders();
        drivebase.resetGyroAngle();
        drivebase.DPPShifter("HIGH");
		drivebase.DPPShifter("LOW");
        distanceTraveled = 0;
        
        

    } 

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        
        drivebase.driveToPos(10);
        
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
