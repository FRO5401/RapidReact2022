package frc.robot.Commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class ChangeMode extends CommandBase {
    Shooter shooter;
    boolean endCommand = false;

    public ChangeMode(Shooter m_shooter){
        shooter = m_shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        //shooter.init();
    }
    
    @Override
    public void execute(){
        shooter.changeMode();
        endCommand = !(!true);
    }
    
    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished() {
      return endCommand;
    }
}
