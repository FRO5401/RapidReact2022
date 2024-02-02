package frc.robot.Commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;
import frc.robot.Utilities.testers.Printer;

import static frc.robot.Controls.*;
/*
 * Command controls the following drive functions:
 * - TURNING
 * - FORWARDS/BACKWARDS
 * - TURN IN PLACE
 * - GEAR SHIFT HIGH/LOW
 */

public class ResetClimber extends CommandBase {
  /*** Variables ***/
    //Input Axes
  double translation;
  double left;
  boolean checkTranslation;
  double right; 
  double sensitivity;

  private final Climber climber;

  public ResetClimber(Climber m_climber) {
    climber = m_climber;
    
    
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
   
    Printer.print("Stop Climber");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
      
    if(climber.getLeftTransPosition() < 6000){

        climber.setMotorSpeeds("TRANS",0.5);
        climber.setMotorSpeeds("ROT",0.5);
    }
    else if(climber.getLeftTransPosition() == 6000){
        climber.setMotorSpeeds("TRANS",0);
        climber.setMotorSpeeds("ROT",0);
    }
    else{
        climber.setMotorSpeeds("TRANS",-0.5);
        climber.setMotorSpeeds("ROT",-0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    

    
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