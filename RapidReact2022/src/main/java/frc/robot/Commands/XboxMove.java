package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveBase;
import frc.robot.RobotContainer;

/*
 * Command controls the following drive functions:
 * - TURNING
 * - FORWARDS/BACKWARDS
 * - TURN IN PLACE
 * - GEAR SHIFT HIGH/LOW
 */

public class XboxMove extends CommandBase {
  /*** Variables ***/
    //Input Axes
  double turn;
  double throttle;
  double reverse;

    //Input Buttons
  boolean rotate; 
  boolean brake;
  boolean precision;
  boolean gearShiftHigh;
  boolean gearShiftLow;
  boolean shoot;
     //Testing Buttons (TODO: Remove for Comp)
  boolean resetSensors;
  /*
  boolean speedConstant1;
  boolean speedConstant2;
  boolean speedConstant3;
 */
    //Instance Vars
  double left;
  double right; 
  double sensitivity;

  private final DriveBase drivebase;

  public XboxMove(DriveBase m_drivebase) {
    drivebase = m_drivebase;
    
    addRequirements(drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    drivebase.shiftHighToLow();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    /*** Read Inputs ***/
      //Axes
    turn = Controls.xboxAxis(Controls.driver, "LS-X");
    throttle = Controls.xboxAxis(Controls.driver, "RT");
    reverse = Controls.xboxAxis(Controls.driver, "LT");
      //Buttons
    shoot = Controls.xboxButton(Controls.operator, "A");
    rotate = Controls.xboxButton(Controls.driver, "LS");
    brake = Controls.xboxButton(Controls.driver, "LB");
    precision = Controls.xboxButton(Controls.driver, "RB");
    gearShiftHigh = Controls.xboxButton(Controls.driver, "START");
    gearShiftLow = Controls.xboxButton(Controls.driver, "BACK");

    resetSensors = Controls.xboxButton(Controls.operator, "START");
    
      //TODO: Remove these testing buttons for competition.
    /*resetSensors
    speedConstant1 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_X);
    speedConstant2 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_A);
    speedConstant3 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_B);
      //TODO: Remove this testing method for competition.
    */
    if(resetSensors){
      drivebase.resetEncoders();
      drivebase.resetGyroAngle();
      drivebase.resetTalon(true);
      drivebase.resetTalon(false); // ;)
      System.out.println("reset sensors");
    }    
     
    /*** Gear Shifting ***/
      //Press for High Gear
    /*if(gearShiftHigh){
        drivebase.shiftLowToHigh();
    }
      //Press for Low Gear
    else */if(gearShiftLow){
        drivebase.shiftHighToLow();
    }

    /*** Precision ***/
      //Hold for Precision Speed
    if(precision){
      sensitivity = Constants.ControlConstants.DRIVE_SENSITIVITY_PRECISION;
    }
      //Release for Regular Speed
    else{
      sensitivity = Constants.ControlConstants.DRIVE_SENSITIVITY_DEFAULT;
    }

    /*** Driving ***/
      //Braking
    if(brake){
      //Robot.drivebase.stopMotors();
      left = 0;
      right = 0;
    }/* 
      //TODO: Remove these testing conditionals for competition. 
    else if(speedConstant1){
      left = (1.0 / 3);
      right = (1.0 / 3);
    }
    else if(speedConstant2){
      left = (2.0 / 3);
      right = (2.0 / 3);
    }
    else if(speedConstant3){
      left = (1.0);
      right = (1.0);
    } */
      //Not Braking
    else{
        //Pirouetting (Turn in place). 
      if(rotate){
          //If the joystick is pushed passed the threshold. 
        if(Math.abs(turn) > Constants.ControlConstants.AXIS_THRESHOLD){
            //Sets it to spin the desired direction.
          left = Constants.ControlConstants.SPIN_SENSITIVITY * turn;
          right = Constants.ControlConstants.SPIN_SENSITIVITY * (turn * -1);
        }
          //If its not past the threshold stop spinning
        else if(Math.abs(turn) < Constants.ControlConstants.AXIS_THRESHOLD){
          left = 0;
          right = 0;
        }
      }
        //Not pirouetting (Not turning in place).
      else{
          //Turning right
        if(turn > Constants.ControlConstants.AXIS_THRESHOLD){
            //Makes left slow down by a factor of how far the axis is pushed. 
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity * (1 - turn);
        }
          //Turning left
        else if(turn < (-1 * Constants.ControlConstants.AXIS_THRESHOLD)){
            //Makes right speed up by a factor of how far the axis is pushed. 
          left = (throttle - reverse) * sensitivity  * (1 + turn);
          right = (throttle - reverse) * sensitivity;
        }
          //Driving straight 
        else{
            //No joystick manipulation. 
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity;
        }
      }
    }
    
      //After speed manipulation, send to drivebase. 
    drivebase.drive(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0,0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
 
  @Override
    public boolean runsWhenDisabled() {
      return false;
  }
}