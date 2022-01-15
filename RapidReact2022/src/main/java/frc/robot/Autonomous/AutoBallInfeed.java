package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoBallInfeed extends Command {

    private double desiredDistance;
    private double currentAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;
	private double radius;
	private double ballLocation;
    
    private double startTime;
    private double currentTime;
    
    private boolean isCentered;

	public AutoBallInfeed(double SpeedInput) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Robot.drivebase);

		/*try {
			desiredDistance = Robot.networktables.getBallDistance();
		}
		catch (NullPointerException e)
		{
			desiredDistance = 0;
		}	*/

		autoDriveSpeed = SpeedInput;
		distanceTraveled = 0;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		startTime = Timer.getMatchTime();
		
		Robot.networktables.resetValues();
		Robot.drivebase.resetSensors();
		Robot.drivebase.setDPPHighGear();
		Robot.drivebase.setDPPLowGear();

		ballLocation = Robot.networktables.getBXValue();

		doneTraveling = false;
		isCentered = false;
		distanceTraveled = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		currentTime = Timer.getMatchTime();
		double timeElapsed = startTime - currentTime;
        SmartDashboard.putNumber("Time elapsed", timeElapsed);

		radius = Robot.networktables.getBallRadius();

		if(isCentered == false){
			isCentered = Robot.networktables.checkCentered();
			currentAngle = Robot.networktables.getBXValue();
			try {
				desiredDistance = Robot.networktables.getBallDistance();
			}
			catch (NullPointerException e) {
				desiredDistance = 0;
			} 
		}
		
		if(radius == 0){ //If no ball is recognized, scan area
			isCentered = false;
			if(timeElapsed >= 3){//If no ball has been found after 3 seconds, go back to original angle and stop
				if(Robot.drivebase.navxGyro.getAngle() > (Robot.drivebase.navxGyro.getAngle() % 366)){
					Robot.drivebase.drive(-1 * autoDriveSpeed, autoDriveSpeed);
				}
				else if(Robot.drivebase.navxGyro.getAngle() < (Robot.drivebase.navxGyro.getAngle() % 366)){
					Robot.drivebase.drive(autoDriveSpeed, -1 * autoDriveSpeed);
				}
				else{
					Robot.drivebase.stopMotors();
					doneTraveling = true;
					Robot.drivebase.navxGyro.reset();
				}	
			}
			else if((timeElapsed) < 3){
				Robot.drivebase.drive(0.2, (-1 * 0.2));
			}
		}
		else if(Robot.networktables.radius > 0){ //If ball is recognized drive towards it and infeed
		    if(isCentered == true) { //Once recognized ball is straight ahead, drive towards it based off of received distance
				//Robot.infeed.startMotors();

				if(radius < 200){
					Robot.drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, Robot.drivebase.navxGyro.getAngle());
				}
				else{
		    	    Robot.drivebase.stopMotors();
					doneTraveling = true;
				}
            }
    	    else { //Turn until the ball that is recognized is straight ahead
			    if(currentAngle < ballLocation){
				    Robot.drivebase.drive(autoDriveSpeed, (-1 * autoDriveSpeed));
			    }
        	    else if(currentAngle > ballLocation){
				    Robot.drivebase.drive((-1 * autoDriveSpeed), autoDriveSpeed);
			    }
            }
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		System.out.print("Should be finished");
		return doneTraveling;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivebase.stopMotors();
		// System.out.println("Angle when EXITING DriveShift:" +
		// Robot.drivebase.getGyroAngle());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.drivebase.stopMotors();
	}

}