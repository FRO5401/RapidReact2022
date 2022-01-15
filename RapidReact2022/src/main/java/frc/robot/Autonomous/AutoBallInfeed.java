package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.NetworkTables;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoBallInfeed extends CommandBase {

    private double desiredDistance;
    private double currentAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;
	private double radius;
	private double ballLocation;
	private DriveBase drivebase;
	private NetworkTables networktables;
    
    private double startTime;
    private double currentTime;
    
    private boolean isCentered;

	public AutoBallInfeed(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);

		/*try {
			desiredDistance = networktables.getBallDistance();
		}
		catch (NullPointerException e)
		{
			desiredDistance = 0;
		}	*/

		autoDriveSpeed = SpeedInput;
		distanceTraveled = 0;
		drivebase = passedDrivebase;
		networktables = passedNetworkTables;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {

		startTime = Timer.getMatchTime();
		
		networktables.resetValues();
		drivebase.resetEncoders();
		drivebase.resetGyroAngle();
		drivebase.setDPPHighGear();
		drivebase.setDPPLowGear();

		ballLocation = networktables.getBXValue();

		doneTraveling = false;
		isCentered = false;
		distanceTraveled = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		currentTime = Timer.getMatchTime();
		double timeElapsed = startTime - currentTime;
        SmartDashboard.putNumber("Time elapsed", timeElapsed);

		radius = networktables.getBallRadius();

		if(isCentered == false){
			isCentered = networktables.checkCentered();
			currentAngle = networktables.getBXValue();
			try {
				desiredDistance = networktables.getBallDistance();
			}
			catch (NullPointerException e) {
				desiredDistance = 0;
			} 
		}
		
		if(radius == 0){ //If no ball is recognized, scan area
			isCentered = false;
			if(timeElapsed >= 3){//If no ball has been found after 3 seconds, go back to original angle and stop
				if(drivebase.getGyroAngle() > (drivebase.getGyroAngle() % 366)){
					drivebase.drive(-1 * autoDriveSpeed, autoDriveSpeed);
				}
				else if(drivebase.getGyroAngle() < (drivebase.getGyroAngle() % 366)){
					drivebase.drive(autoDriveSpeed, -1 * autoDriveSpeed);
				}
				else{
					drivebase.drive(0,0);
					doneTraveling = true;
					drivebase.resetGyroAngle();
				}	
			}
			else if((timeElapsed) < 3){
				drivebase.drive(0.2, (-1 * 0.2));
			}
		}
		else if(networktables.radius > 0){ //If ball is recognized drive towards it and infeed
		    if(isCentered == true) { //Once recognized ball is straight ahead, drive towards it based off of received distance
				//infeed.startMotors();

				if(radius < 200){
					drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, drivebase.getGyroAngle());
				}
				else{
		    	    drivebase.drive(0,0);
					doneTraveling = true;
				}
            }
    	    else { //Turn until the ball that is recognized is straight ahead
			    if(currentAngle < ballLocation){
				    drivebase.drive(autoDriveSpeed, (-1 * autoDriveSpeed));
			    }
        	    else if(currentAngle > ballLocation){
				    drivebase.drive((-1 * autoDriveSpeed), autoDriveSpeed);
			    }
            }
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		System.out.print("Should be finished");
		return doneTraveling;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		drivebase.drive(0,0);
		// System.out.println("Angle when EXITING DriveShift:" +
		// drivebase.getGyroAngle());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
}