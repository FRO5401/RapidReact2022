package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.*;
import frc.robot.Commands.TestMotor;
import frc.robot.Commands.TestSolenoid;
import frc.robot.Subsystems.CompressorSubsystem;
import frc.robot.Subsystems.MotorTester;
import frc.robot.Subsystems.SolenoidTester;

public class RobotContainer {
    
    private final SendableChooser<Command> chooser = new SendableChooser<Command>();
    // The robot's subsystems
    private final MotorTester motortester = new MotorTester();
    private final SolenoidTester solenoidtester = new SolenoidTester();
    private final Controls controls = new Controls();
    private final CompressorSubsystem compressor = new CompressorSubsystem();


    public RobotContainer() {
        configureButtonBindings();
        chooser.setDefaultOption("Do Nothing", new DoNothing());
        chooser.addOption("Drive Straight", new DriveStraight());
        SmartDashboard.putData("Auto choices", chooser);
        motortester.setDefaultCommand(new TestMotor(motortester, controls));
        solenoidtester.setDefaultCommand(new TestSolenoid(solenoidtester, controls));
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand(){
        return chooser.getSelected();
    }

}
