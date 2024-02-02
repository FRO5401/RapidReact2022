package frc.robot.Autonomous.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Climber;
import java.util.Date;
/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoTranslateArm extends CommandBase {
    private double speed;
	private long executionPeriod;
	private Climber climber;
	private Date startTime;
	private boolean doneTranslating;
	public AutoTranslateArm(double SpeedInput, Climber passedClimber, double passedTime) {
        speed = SpeedInput;
		climber = passedClimber;
		
		executionPeriod = (long)(passedTime*1000);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);

	
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		startTime = new Date();
		doneTranslating = false;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		Date currentTime = new Date();
		long timeElapsed = currentTime.getTime() - startTime.getTime();
		if(timeElapsed < executionPeriod) climber.setMotorSpeeds("TRANS", speed);
		else doneTranslating = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return doneTranslating;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		climber.setMotorSpeeds("TRANS", 0);
	}

	@Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
}