package frc.robot.Commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class LoadBall extends CommandBase {
    Shooter shooter;
    boolean endCommand = false;
    String mode;

    public LoadBall(Shooter m_shooter, String mode){
        shooter = m_shooter;
        this.mode = mode;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        //shooter.init();
    }
    
    @Override
    public void execute(){
        shooter.load(mode);
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
