package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.RobotMap;
import frc.robot.Subsystems.MotorTester;


public class TestMotor extends CommandBase {
  /*** Variables ***/
  //Input Axes
  double axis3;
  double axis2;
  double axis1;

  //Input Buttons
  boolean button1;
  boolean button2;
  boolean button3;

  private final MotorTester motortester;
  private final Controls controls;

  public TestMotor(MotorTester newMotorTester, Controls newControls) {
    motortester = newMotorTester;
    controls = newControls;
    
    addRequirements(motortester);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    /*** Read Inputs ***/
      //Axes
    axis1 = controls.xboxAxis(controls.xboxDriver, RobotMap.XBOX_AXIS_LEFT_X);
    //axis2 = controls.xboxAxis(controls.xboxDriver, RobotMap.XBOX_AXIS_RIGHT_TRIGGER);
    //axis3 = controls.xboxAxis(controls.xboxDriver, RobotMap.XBOX_AXIS_LEFT_TRIGGER);
    
      //Buttons
    button1 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_L3);
    //button2 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    //button3 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

}