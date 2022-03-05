package frc.robot.Autonomous.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Climber;


/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoRotateArm extends CommandBase {
    double speed, angle, executionPeriod;
	private Climber climber;
	private double startTime;
    private double currentTime;
	private boolean doneRotating;
	public AutoRotateArm(double SpeedInput, double AngleInput, Climber passedClimber, double executionPeriod) {

        speed = SpeedInput;
        angle = AngleInput;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);

	
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		startTime = Timer.getMatchTime();
		doneRotating = false;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		currentTime = Timer.getMatchTime();
		double timeElapsed = startTime - currentTime;
		if(timeElapsed < executionPeriod){
			climber.setMotorSpeeds("TRANS", speed);
		}
		else{
			doneRotating = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return doneRotating;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		
	}

	@Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
}