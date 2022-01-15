package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.Subsystems.MotorTester;

public class AutoMotor extends CommandBase {

    private MotorTester motortester;
	private double time;
	private double input;
	private boolean doneTraveling;
	private double distanceTraveled;
    private String mode;
    private int motorNum;

	public AutoMotor(int MotorInput, String ModeInput, double SpeedInput, double TimeInput, MotorTester passedmotortester) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);

        motorNum = MotorInput;
        mode = ModeInput;
        motortester = passedmotortester;
		time = TimeInput;
		input = SpeedInput;
		doneTraveling = true;
        addRequirements(motortester);
	}

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        doneTraveling = false;
    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        motortester.runMotor(input, mode, motorNum);
        new WaitCommand(time);
        motortester.runMotor(0, mode, motorNum);
        doneTraveling = true;
    }

    // Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
        motortester.runMotor(0, mode, motorNum);
	}

	// Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return doneTraveling;
    }
}