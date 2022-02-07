package frc.robot.Commands.internal_mech;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.InternalMech;

public class RunBelt extends CommandBase{
     /*** Variables ***/

  InternalMech internalMech;   
  int angle;

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
        internalMech.run("PULL");
    }  else if (angle == 180) {
        internalMech.run("PUSH");
    } else {
        internalMech.run("STOP");
    }
  }

  @Override
  public void end(boolean interrupted) {
    internalMech.run("STOP");
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