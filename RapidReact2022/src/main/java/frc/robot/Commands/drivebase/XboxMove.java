package frc.robot.Commands.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

  public XboxMove(DriveBase m_drivebase, DoubleSupplier newThrottle, DoubleSupplier newReverse, DoubleSupplier newTurn, BooleanSupplier newRotate, BooleanSupplier newPrecision, BooleanSupplier newBrake) {
    drivebase = m_drivebase;
    throttle = newThrottle.getAsDouble();
    reverse = newReverse.getAsDouble();
    turn = newTurn.getAsDouble();
    precision = newPrecision.getAsBoolean();
    brake = newBrake.getAsBoolean();
    rotate = newRotate.getAsBoolean();
    
    
    addRequirements(drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    drivebase.shift("LOW");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
      //Braking
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
    }   
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