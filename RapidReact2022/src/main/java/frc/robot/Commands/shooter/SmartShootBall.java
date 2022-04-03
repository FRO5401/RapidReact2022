package frc.robot.Commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;

public class SmartShootBall extends CommandBase {
    Shooter shooter;
    double speed;
    boolean endCommand = false;

    public SmartShootBall(Shooter m_shooter, double setSpeed){
        shooter = m_shooter;
        speed = setSpeed;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START");
        shooter.runSmart(""+speed);
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
