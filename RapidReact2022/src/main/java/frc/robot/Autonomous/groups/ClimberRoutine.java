package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Climber;
import frc.robot.Autonomous.actions.*;
import frc.robot.Commands.climber.*;

//TODO: Climber stages for override need to be done later
public class ClimberRoutine extends SequentialCommandGroup {
    public ClimberRoutine(Climber passedClimber) {
        addCommands(
            //We need to figure out the time we rotate the climber for
            //We need to figure out how long to translate the climber for
        );
    }
}