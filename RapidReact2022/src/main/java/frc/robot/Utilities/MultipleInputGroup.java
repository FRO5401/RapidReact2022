package frc.robot.Utilities;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MultipleInputGroup extends Trigger{
    ArrayList<JoystickAxis> joystickInputs;
    ArrayList<JoystickButton> joystickButtonInputs;

    public MultipleInputGroup(){

    }

    public void addAxis(JoystickAxis axis){
        joystickInputs.add(axis);
    }

    public void addButton(JoystickButton button){
        joystickButtonInputs.add(button);
    }

    public void whenAnyActive(Command command){
        whileActiveContinuous(command);
    }

    public void whenAllInactive(Command command){
        whenInactive(command);
    }

    @Override
    public boolean get(){
        boolean anyPressed;
        for (JoystickButton button : joystickButtonInputs) {
            if(button.get()==true){
                anyPressed = true;
                return anyPressed;
            }
        }
        for (JoystickAxis axis : joystickInputs){
            if(axis.get()==true){
                anyPressed = true;
                return anyPressed;
            }
        }
        return false;
    }

}
