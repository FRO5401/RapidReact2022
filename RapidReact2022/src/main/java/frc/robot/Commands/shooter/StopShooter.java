package frc.robot.Commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class StopShooter extends CommandBase {
    Shooter shooter;
    boolean endCommand = false;

    public StopShooter(Shooter m_shooter){
        shooter = m_shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        shooter.run("STOP");
        endCommand = true;
    }
    
    @Override
    public void end(boolean interrupted){
        
    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}
