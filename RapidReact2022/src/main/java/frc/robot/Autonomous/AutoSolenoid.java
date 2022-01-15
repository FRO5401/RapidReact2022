package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.Subsystems.SolenoidTester;

public class AutoSolenoid extends CommandBase {

    private SolenoidTester solenoidtester;
	private double time;
	private boolean input;
    private int solenoid;
	private boolean doneTraveling;

	public AutoSolenoid(int SolenoidInput, boolean StateInput, double TimeInput, SolenoidTester passedsolenoidtester) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);
        solenoidtester = passedsolenoidtester;
		input = StateInput;
        time = TimeInput;
        solenoid = SolenoidInput;
		doneTraveling = true;
        addRequirements(solenoidtester);
	}

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        doneTraveling = false;
    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        solenoidtester.activateSolenoids(input, solenoid);
        new WaitCommand(time);
        solenoidtester.activateSolenoids(!input, solenoid);
        doneTraveling = true;
    }

    // Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		solenoidtester.activateSolenoids(!input, solenoid);
	}

	// Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return doneTraveling;
    }
}