package frc.robot.Autonomous.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
            
            
            //Start climb to Mid
            new AutoTranslateArm(1,passedClimber,4.20), //+dir
            new AutoTranslateArm(-1,passedClimber,4.20), //-dir 
            new AutoRotateArm(1,passedClimber,2), //+dir
            new AutoRotateArm(-1,passedClimber,2)//, //-dir ONLY HERE FOR TESTING
            /*
            new ParallelCommandGroup(
                new AutoTranslateArm(), -dir
                new AutoRotateArm(), slower speed this time to try to grip on -dir
            ),
            new WaitCommand(),
            
            //Trans Unhook Mid
            new ParallelCommandGroup(
                new AutoRotateArm(), +dir
                new AutoTranslateArm() to unhook +dir
            ),

            //Trans Clearing High
            new AutoTranslateArm(), -dir
            
            //Rotating to High
            new AutoRotateArm(), +dir
            could be in a parallel group with clearing, safer not to be 
            
            //Trans Reach High
            new AutoTranslateArm(), +dir

            //Trans Grab High
            new ParallelCommandGroup(
                new AutoTranslateArm(), -dir //Not all the way
                new AutoRotateArm(), slower speed this time to try to grip on -dir
            ),

            //Static Unhook Mid
            new AutoRotateArm(), +dir

            //Static Clear High (if needed)
            new AutoTranslateArm(), +dir

            //Static Reach and Grab High
            new ParallelCommandGroup(
                new AutoTranslateArm(), -dir //Reach behind High
                new AutoRotateArm(), slower speed this time to try to grip on -dir //Grab High
            ),

            //Trans Unhook High
            new ParallelCommandGroup(
                new AutoRotateArm(), +dir
                new AutoTranslateArm() to unhook +dir
            ),

            //Trans Clearing Trav
            new AutoTranslateArm(), -dir
            
            //Rotating to Trav
            new AutoRotateArm(), +dir
            could be in a parallel group with clearing, safer not to be 
            
            //Trans Reach Trav
            new AutoTranslateArm(), +dir

            //Trans Grab Trav
            new ParallelCommandGroup(
                new AutoTranslateArm(), -dir //Not all the way
                new AutoRotateArm(), slower speed this time to try to grip on -dir
            ),

            //Static Unhook High
            new AutoRotateArm(), +dir

            //Static Clear Trav (if needed)
            new AutoTranslateArm(), +dir

            //Static Reach and Grab Trav
            new ParallelCommandGroup(
                new AutoTranslateArm(), -dir //Reach behind High
                new AutoRotateArm(), slower speed this time to try to grip on -dir //Grab High
            ),

            from here just keep adding parallel command groups of rotating and translating the arms.
            */
        );
    }
}