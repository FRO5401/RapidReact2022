package frc.robot.Utilities.controllers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DPad extends Trigger{
    public DPad whenHeld(final Command command, boolean interruptible) {
        whileActiveContinuous(command, interruptible);
        return this;
    }

    public DPad whenHeld(final Command command) {
        whileActiveContinuous(command);
        return this;
    }

    public DPad whenPressed(final Command command, boolean interruptible) {
        whenActive(command, interruptible);
        return this;
    }

    public DPad whenPressed(final Command command) {
        whenActive(command);
        return this;
    }

    public DPad whenReleased(final Command command, boolean interruptible){
        whenReleased(command, interruptible);
        return this;
    }

    public DPad whenReleased(final Command command){
        whenReleased(command);
        return this;
    }
}
