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

public class MoveClimberArms extends CommandBase {
  /*** Variables ***/
    //Input Axes
  double translation;
  boolean checkTranslation;
  double sensitivity;
  double rotation;
  boolean checkRotation;
  boolean checkOverExtension;

  private final Climber climber;

  public MoveClimberArms(Climber m_climber) {
    climber = m_climber;
    
    
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
   
    Printer.print("Climber-Trans");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    translation = xboxAxis(operator, "RS-Y").getAxis();
    checkTranslation = xboxAxis(operator, "RS-Y").get();
    if(checkTranslation && !climber.checkOverExtension(climber.posToAngle((int)climber.getLeftRotAngle()), (int)climber.getLeftTransPosition()))
        climber.setMotorSpeeds("TRANS", translation);
    else
        climber.setMotorSpeeds("TRANS", 0);


    rotation = xboxAxis(operator, "LS-X").getAxis();
    checkRotation = xboxAxis(operator, "LS-X").get();
    if(checkRotation && !climber.checkOverExtension(climber.posToAngle((int)climber.getLeftRotAngle()), (int)climber.getLeftTransPosition()))
        climber.setMotorSpeeds("ROT", rotation);    
    else
        climber.setMotorSpeeds("ROT", 0);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotorSpeeds("TRANS",0);
    climber.setMotorSpeeds("ROT",0);
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