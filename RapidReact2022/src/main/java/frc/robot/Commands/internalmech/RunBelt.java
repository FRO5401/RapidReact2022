package frc.robot.Commands.internalmech;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.InternalMech;

public class RunBelt extends CommandBase{
     /*** Variables ***/

  InternalMech internalMech;   
  int angle;
  boolean endCommand = false;

  public RunBelt(InternalMech m_internalMech, IntSupplier newAngle) {
    internalMech = m_internalMech;
    angle = newAngle.getAsInt();

    addRequirements(internalMech);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(angle == 0){
        internalMech.mechPull();
    }  else if (angle == 180) {
        internalMech.mechPush();
    } else {
        internalMech.setMechSpeeds(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    internalMech.setMechSpeeds(0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return endCommand;
  }

  @Override
    public boolean runsWhenDisabled() {
      return false;
  }
}

