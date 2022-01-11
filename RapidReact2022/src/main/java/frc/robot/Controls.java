package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controls {
      // The driver's controller
      public XboxController xboxDriver = new XboxController(RobotMap.XBOX_CONTROLLER_DRIVER);
      public XboxController xboxOperator = new XboxController(RobotMap.XBOX_CONTROLLER_OPERATOR);
    
      //Buttons (Driver)
      JoystickButton xboxA_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_A);
      JoystickButton xboxB_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_B);
      JoystickButton xboxX_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_X);
      JoystickButton xboxY_Driver			  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_Y);
      JoystickButton xboxLeftBumper_Driver  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
      JoystickButton xboxRightBumper_Driver = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
      JoystickButton xboxBack_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_BACK);
      JoystickButton xboxStart_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_START);
      JoystickButton xboxL3_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_L3);
      JoystickButton xboxR3_Driver		  = new JoystickButton(xboxDriver, RobotMap.XBOX_BUTTON_R3);
      
        //Buttons (Operator)
      JoystickButton xboxA_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_A);
      JoystickButton xboxB_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_B);
      JoystickButton xboxX_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_X);
      JoystickButton xboxY_Operator			= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_Y);
      JoystickButton xboxLeftBumper_Operator  = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_LEFT_BUMPER);
      JoystickButton xboxRightBumper_Operator = new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_RIGHT_BUMPER);
      JoystickButton xboxBack_Operator		= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_BACK);
      JoystickButton xboxStart_Operator		= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_START);
      JoystickButton xboxL3_Operator		  	= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_L3);
      JoystickButton xboxR3_Operator		  	= new JoystickButton(xboxOperator, RobotMap.XBOX_BUTTON_R3);
    
      public double xboxAxis(XboxController xboxController, int xboxAxis){
        return xboxController.getRawAxis(xboxAxis);
      }
    
      public boolean xboxButton(XboxController xboxController, int xboxButton){
        return xboxController.getRawButton(xboxButton);
      }
    
      public int xboxAxisAsButton(XboxController xboxController, int xboxAxis){
        if(xboxController.getRawAxis(xboxAxis) > RobotMap.AXIS_THRESHOLD){
          return 1;
        }
        else if(xboxController.getRawAxis(xboxAxis) < (-1 * RobotMap.AXIS_THRESHOLD)){
          return -1;
        }
        else{
          return 0;
        }
      }
    
      public int xboxDPad(XboxController xboxController){
        return xboxController.getPOV();
      }
}