package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.RobotMap;
import frc.robot.Subsystems.DriveBase;

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
  private final Controls controls;

  public XboxMove(DriveBase m_drivebase, Controls m_controls) {
    drivebase = m_drivebase;
    controls = m_controls;
    
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
    turn = controls.xboxAxis(controls.xboxDriver, RobotMap.XBOX_AXIS_LEFT_X);
    throttle = controls.xboxAxis(controls.xboxDriver, RobotMap.XBOX_AXIS_RIGHT_TRIGGER);
    reverse = controls.xboxAxis(controls.xboxDriver, RobotMap.XBOX_AXIS_LEFT_TRIGGER);
    
      //Buttons
    rotate = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_L3);
    brake = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
    precision = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
    gearShiftHigh = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_START);
    gearShiftLow = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_BACK);

    resetSensors = controls.xboxButton(controls.xboxOperator, RobotMap.XBOX_BUTTON_START);
    
      //TODO: Remove these testing buttons for competition.
    /*resetSensors
    speedConstant1 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_X);
    speedConstant2 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_A);
    speedConstant3 = controls.xboxButton(controls.xboxDriver, RobotMap.XBOX_BUTTON_B);
      //TODO: Remove this testing method for competition.
    */
   /* if(resetSensors){
      drivebase.resetSensors();
      System.out.println("reset sensors");
    }    */
     
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
      sensitivity = RobotMap.DRIVE_SENSITIVITY_PRECISION;
    }
      //Release for Regular Speed
    else{
      sensitivity = RobotMap.DRIVE_SENSITIVITY_DEFAULT;
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
        if(Math.abs(turn) > RobotMap.AXIS_THRESHOLD){
            //Sets it to spin the desired direction.
          left = RobotMap.SPIN_SENSITIVITY * turn;
          right = RobotMap.SPIN_SENSITIVITY * (turn * -1);
        }
          //If its not past the threshold stop spinning
        else if(Math.abs(turn) < RobotMap.AXIS_THRESHOLD){
          left = 0;
          right = 0;
        }
      }
        //Not pirouetting (Not turning in place).
      else{
          //Turning right
        if(turn > RobotMap.AXIS_THRESHOLD){
            //Makes left slow down by a factor of how far the axis is pushed. 
          left = (throttle - reverse) * sensitivity;
          right = (throttle - reverse) * sensitivity * (1 - turn);
        }
          //Turning left
        else if(turn < (-1 * RobotMap.AXIS_THRESHOLD)){
            //Makes right speed up by a factor of how far the axis is pushed. 
          left = (throttle - reverse) * sensitivity * (1 + turn);
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

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

}