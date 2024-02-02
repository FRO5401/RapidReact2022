package frc.robot.Autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveBase;

public class AutoTurn extends CommandBase {

    private DriveBase drivebase;
	private double desiredAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
    private double turnStartTime, turnCurrentTime;
    private int count;

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
        drivebase.resetEncoders();
        drivebase.DPPShifter("HIGH");
		drivebase.DPPShifter("LOW");
        drivebase.resetGyroAngle();
        doneTraveling = false;
        count = 0;
    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        
        drivebase.autoTurn(autoDriveSpeed*0.9, desiredAngle);

        if(drivebase.getGyroYaw() < desiredAngle+Constants.AutoConstants.ANGULAR_THRESHOLD && drivebase.getGyroYaw() > desiredAngle-Constants.AutoConstants.ANGULAR_THRESHOLD)
            doneTraveling = true;
/** 
        if(drivebase.getGyroAngle() < desiredAngle && drivebase.getGyroAngle() > desiredAngle){
            if(count == 0){
                turnStartTime  = Timer.getFPGATimestamp();
                count++;
            }
            
            
            if(Timer.getFPGATimestamp() - turnStartTime >  0.05)
                doneTraveling =true;
        }
        else{
            count = 0;
        }
*/
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