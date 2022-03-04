package frc.robot.Commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Utilities.testers.Printer;
import frc.robot.RobotContainer;

import static frc.robot.Controls.*;
/*
 * Command controls the following drive functions:
 * - TURNING
 * - FORWARDS/BACKWARDS
 * - TURN IN PLACE
 * - GEAR SHIFT HIGH/LOW
 */

public class TranslateClimberArms extends CommandBase {
  /*** Variables ***/
    //Input Axes
  double translation;
  double left;
  boolean checkTranslation;
  double right; 
  double sensitivity;

  private final Climber climber;

  public TranslateClimberArms(Climber m_climber) {
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
    if(checkTranslation)
        climber.setMotorSpeeds("TRANS", translation);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotorSpeeds("TRANS",0);
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