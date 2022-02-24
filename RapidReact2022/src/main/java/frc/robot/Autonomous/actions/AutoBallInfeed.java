package frc.robot.Autonomous.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Infeed;
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
	private Infeed infeed;
    
    private double startTime;
    private double currentTime;
    
    private boolean isCentered;

	public AutoBallInfeed(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables, Infeed passedInfeed) {
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
		infeed = passedInfeed;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		
		startTime = Timer.getMatchTime();
		System.out.println("Baller");
		System.out.println("Baller");
		System.out.println("Baller");
		System.out.println("Baller");
		System.out.println("Baller");
		System.out.println("Baller");
		networktables.resetValues();
		drivebase.resetEncoders();
		drivebase.resetGyroAngle();
		drivebase.DPPShifter("HIGH");
		drivebase.DPPShifter("LOW");

		doneTraveling = false;
		isCentered = false;
		distanceTraveled = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		ballLocation = networktables.getBallBXValue();
		currentTime = Timer.getMatchTime();
		double timeElapsed = startTime - currentTime;
        SmartDashboard.putNumber("Time elapsed", timeElapsed);
		radius = networktables.getBallRadius();

		if(isCentered == false){
			isCentered = networktables.checkCentered();
			currentAngle = drivebase.getGyroAngle();
			try {
				desiredDistance = networktables.getBallDistance()*1000;
			}
			catch (NullPointerException e) {
				desiredDistance = 0;
			} 
		}
		
		if(radius == 0){ //If no ball is recognized, scan area
			isCentered = false;
			if(timeElapsed >= 3){//If no ball has been found after 3 seconds, go back to original angle and stop
				if(drivebase.getGyroAngle() > (drivebase.getGyroAngle() % 366)){
					drivebase.drive(autoDriveSpeed, autoDriveSpeed);
				}
				else if(drivebase.getGyroAngle() < (drivebase.getGyroAngle() % 366)){
					drivebase.drive(autoDriveSpeed, autoDriveSpeed);
				}
				else{
					drivebase.drive(0,0);
					doneTraveling = true;
					drivebase.resetGyroAngle();
				}	
			}
			else if((timeElapsed) < 3){
				drivebase.drive(0.2, 0.2);
			}
		}
		else if(radius > 0){ //If ball is recognized drive towards it and infeed
		    if(isCentered == true) { //Once recognized ball is straight ahead, drive towards it based off of received distance
				infeed.run("START");

				if(radius < 200){
					drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, drivebase.getGyroAngle());
				}
				else{
		    	    drivebase.drive(0,0);
					doneTraveling = true;
				}
            }
    	    else { //Turn until the ball that is recognized is straight ahead
			    if((currentAngle+320) < ballLocation){
				    drivebase.autoVisionTurn(autoDriveSpeed);
					System.out.println(isCentered);
					System.out.println("Bangerang" + ballLocation + "   " + (currentAngle+320));
			    }
        	    else if((currentAngle+320) > ballLocation){
				    drivebase.autoVisionTurn(-autoDriveSpeed);
					System.out.println(isCentered);
					System.out.println("AMobg us" + ballLocation + "   " + (currentAngle+320));
				}
				else {
					drivebase.drive(0, 0);
					System.out.println("Biden, joe");
				}
            }
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
		// System.out.println("Angle when EXITING DriveShift:" +
		// drivebase.getGyroAngle());
	}

	@Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
}