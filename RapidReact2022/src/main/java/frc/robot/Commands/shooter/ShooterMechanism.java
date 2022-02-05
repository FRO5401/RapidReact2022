package frc.robot.Commands.shooter;
import frc.robot.Subsystems.Shooter;
import frc.robot.Controls;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterMechanism extends CommandBase {
    Shooter shooter;
    Controls controls;
    boolean runShooter;

    public ShooterMechanism(Shooter m_shooter){
        shooter = m_shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.init();
    }
    
    public void execute(double speed){
        shooter.getVelocity();
        shooter.runMotors(speed);

    }
    
    public void end(){
        shooter.stop();
    }
    @Override
    public boolean isFinished() {
      return false;
    }
}
