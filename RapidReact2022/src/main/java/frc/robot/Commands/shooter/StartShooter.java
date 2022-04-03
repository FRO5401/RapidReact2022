package frc.robot.Commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class StartShooter extends CommandBase {
    Shooter shooter;
    boolean endCommand = false;

    public StartShooter(Shooter m_shooter){
        shooter = m_shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START");
        shooter.runSmart("START");
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
