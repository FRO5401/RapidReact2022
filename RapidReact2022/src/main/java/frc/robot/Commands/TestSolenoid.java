package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.RobotMap;
import frc.robot.Subsystems.SolenoidTester;


public class TestSolenoid extends CommandBase {
  /*** Variables ***/
  //Input Buttons
  boolean button1;
  boolean button2;
  boolean button3;

  private final SolenoidTester solenoidtester;
  private final Controls controls;

  public TestSolenoid(SolenoidTester newSolenoidTester, Controls newControls) {
    solenoidtester = newSolenoidTester;
    controls = newControls;
    
    addRequirements(solenoidtester);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    /*** Read Inputs ***/
      //Buttons
    button1 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_B);
    //button2 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    //button3 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

}