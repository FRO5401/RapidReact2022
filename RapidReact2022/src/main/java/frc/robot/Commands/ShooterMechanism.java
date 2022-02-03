package frc.robot.Commands;
import frc.robot.Subsystems.Shooter;
import frc.robot.Controls;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterMechanism extends CommandBase {
    Shooter shooter;
    Controls controls;
    boolean runShooter;

    public ShooterMechanism(Shooter m_shooter, Controls m_controls){
        shooter = m_shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.init();
    }
    @Override
    public void execute(){
        shooter.getVelocity();
        shooter.runMotors(0.5);

    }
    
    public void end(){
        shooter.stop();
    }
    @Override
    public boolean isFinished() {
      return false;
    }
}
