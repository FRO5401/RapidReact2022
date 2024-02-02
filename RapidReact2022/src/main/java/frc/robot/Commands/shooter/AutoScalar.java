package frc.robot.Commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Subsystems.Shooter;

public class AutoScalar extends CommandBase{
    
    Shooter shooter;
    NetworkTables networktables;
    String value;
    double distance;
    double speed;
    double slope = ((0.78-0.75)/(140-125.22));
    public AutoScalar(Shooter passedShooter, NetworkTables passedNetworkTables){
        networktables = passedNetworkTables;
        shooter = passedShooter;
    }

    @Override
    public void initialize(){
        networktables.setMode(3);
    }
    
    @Override
    public void execute(){
        distance = networktables.getTargetDistance();
        speed = slope*distance+0.49;
        value = ""+speed;
        shooter.run(value);
    }
    
    @Override
    public void end(boolean interrupted){
        shooter.run("STOP");
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
