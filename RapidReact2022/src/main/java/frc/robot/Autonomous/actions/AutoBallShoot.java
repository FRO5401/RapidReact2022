package frc.robot.Autonomous.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;

/**
 * This command is also used as a "BaselineOnly" command
 */

public class AutoBallShoot extends CommandBase {

    private double desiredDistance;
    private double currentAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;
	private double targetLocation;
	private DriveBase drivebase;
	private NetworkTables networktables;
	private Shooter shooter;
    
    private double startTime;
    private double currentTime;
    
    private boolean isCentered;

	public AutoBallShoot(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables, Shooter passedShooter) {
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
		shooter = passedShooter;
		
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		
		startTime = Timer.getMatchTime();
		networktables.resetValues();
		drivebase.resetEncoders();
		drivebase.resetGyroAngle();
		drivebase.DPPShifter("HIGH");
		drivebase.DPPShifter("LOW");

		doneTraveling = false;
		isCentered = false;
		distanceTraveled = 0;
		networktables.setMode(3); //3 is shoot
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		targetLocation = networktables.getTargetXValue();
		currentTime = Timer.getMatchTime();
		double timeElapsed = startTime - currentTime;
        SmartDashboard.putNumber("Time elapsed", timeElapsed);

		if(isCentered == false){
			isCentered = networktables.checkCentered("TARGET");
			currentAngle = drivebase.getGyroYaw();
			try {
				desiredDistance = networktables.getBallDistance()*1000;
			}
			catch (NullPointerException e) {
				desiredDistance = 0;
			} 
		}
		
		
		else if(targetLocation > 0){ //If ball is recognized drive towards it and infeed
		    if(isCentered == true) { //Once recognized ball is straight ahead, drive towards it based off of received distance
				//infeed.startMotors();
				/** 
				if(targetLocation < 200){
					drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, drivebase.getGyroAngle());

				}
				else{
		    	    drivebase.drive(0,0);
					doneTraveling = true;
				}
				*/
				
            }
    	    else { //Turn until the ball that is recognized is straight ahead
			    if(((currentAngle+180)*640/360)-1 < targetLocation){
				    drivebase.autoTurn(autoDriveSpeed, currentAngle);
					System.out.println(isCentered);
			    }
        	    else if(((currentAngle+180)*640/360)+1 > targetLocation){
				    drivebase.autoTurn(autoDriveSpeed, currentAngle);
					System.out.println(isCentered);
				}
				else {
					drivebase.drive(0, 0);
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
		shooter.run("START");
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