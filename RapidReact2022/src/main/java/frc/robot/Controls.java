package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Utilities.controllers.JoystickAxis;
import frc.robot.Utilities.controllers.JoystickDPad;

public class Controls {
  // The driver's controller
  public static XboxController driver = new XboxController(Constants.ControlConstants.XBOX_CONTROLLER_DRIVER);
  public static XboxController operator = new XboxController(Constants.ControlConstants.XBOX_CONTROLLER_OPERATOR);

   //Buttons (Driver)
   static JoystickButton xboxA_Driver = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_A);
   static JoystickButton xboxB_Driver			  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_B);
   static JoystickButton xboxX_Driver			  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_X);
   static JoystickButton xboxY_Driver			  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_Y);
   static JoystickButton xboxLeftBumper_Driver  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_LEFT_BUMPER);
   static JoystickButton xboxRightBumper_Driver = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_RIGHT_BUMPER);
   static JoystickButton xboxBack_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_BACK);
   static JoystickButton xboxStart_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_START);
   static JoystickButton xboxL3_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_L3);
   static JoystickButton xboxR3_Driver		  = new JoystickButton(driver, Constants.ControlConstants.XBOX_BUTTON_R3);
   
   //Buttons (Operator)
   static JoystickButton xboxA_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_A);
   static JoystickButton xboxB_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_B);
   static JoystickButton xboxX_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_X);
   static JoystickButton xboxY_Operator			= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_Y);
   static JoystickButton xboxLeftBumper_Operator  = new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_LEFT_BUMPER);
   static JoystickButton xboxRightBumper_Operator = new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_RIGHT_BUMPER);
   static JoystickButton xboxBack_Operator		= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_BACK);
   static JoystickButton xboxStart_Operator		= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_START);
   static JoystickButton xboxL3_Operator		  	= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_L3);
   static JoystickButton xboxR3_Operator		  	= new JoystickButton(operator, Constants.ControlConstants.XBOX_BUTTON_R3);

   //Axis (Driver)
  static JoystickAxis xboxRT_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_RIGHT_TRIGGER);
  static JoystickAxis xboxLT_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_LEFT_TRIGGER);
  static JoystickAxis xboxLX_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_LEFT_X);
  static JoystickAxis xboxRX_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_RIGHT_X);
  static JoystickAxis xboxLY_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_LEFT_Y);
  static JoystickAxis xboxRY_Driver = new JoystickAxis(driver, Constants.ControlConstants.XBOX_AXIS_RIGHT_Y);

   //Axis (Operator)
  static JoystickAxis xboxRT_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_RIGHT_TRIGGER);
  static JoystickAxis xboxLT_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_LEFT_TRIGGER);
  static JoystickAxis xboxLX_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_LEFT_X);
  static JoystickAxis xboxRX_Operator= new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_RIGHT_X);
  static JoystickAxis xboxLY_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_LEFT_Y);
  static JoystickAxis xboxRY_Operator = new JoystickAxis(operator, Constants.ControlConstants.XBOX_AXIS_RIGHT_Y);

  //DPad (Driver)
  static JoystickDPad xboxDPad0_Driver = new JoystickDPad(driver, 0);
  static JoystickDPad xboxDPad45_Driver = new JoystickDPad(driver, 45);
  static JoystickDPad xboxDPad90_Driver = new JoystickDPad(driver, 90);
  static JoystickDPad xboxDPad135_Driver = new JoystickDPad(driver, 135);
  static JoystickDPad xboxDPad180_Driver = new JoystickDPad(driver, 180);
  static JoystickDPad xboxDPad225_Driver = new JoystickDPad(driver, 225);
  static JoystickDPad xboxDPad270_Driver = new JoystickDPad(driver, 270);
  static JoystickDPad xboxDPad315_Driver = new JoystickDPad(driver, 315);

  //DPad (Driver)
  static JoystickDPad xboxDPad0_Operator = new JoystickDPad(operator, 0);
  static JoystickDPad xboxDPad45_Operator = new JoystickDPad(operator, 45);
  static JoystickDPad xboxDPad90_Operator = new JoystickDPad(operator, 90);
  static JoystickDPad xboxDPad135_Operator = new JoystickDPad(operator, 135);
  static JoystickDPad xboxDPad180_Operator= new JoystickDPad(operator, 180);
  static JoystickDPad xboxDPad225_Operator = new JoystickDPad(operator, 225);
  static JoystickDPad xboxDPad270_Operator = new JoystickDPad(operator, 270);
  static JoystickDPad xboxDPad315_Operator = new JoystickDPad(operator, 315);

   public static JoystickButton xboxButton(XboxController controller, String button) {
    JoystickButton output;
    if(controller == driver) {
      switch (button.toUpperCase()) {
        case "A":
          output = xboxA_Driver;
          break;
        case "B":
          output = xboxB_Driver;
          break;
        case "X":
          output = xboxX_Driver;
          break;
        case "Y":
          output = xboxY_Driver;
          break;
        case "LS":
          output = xboxL3_Driver;
          break;
        case "RS":
          output = xboxR3_Driver;
          break;
        case "LB":
          output = xboxLeftBumper_Driver;
          break;
        case "RB":
          output = xboxRightBumper_Driver;
          break;  
        case "START":
          output = xboxStart_Driver;
          break;
        case "BACK":
          output = xboxBack_Driver;
          break;
        default:
          output = xboxA_Driver;  
      }
    }
    else
    {
      switch (button.toUpperCase()) {
        case "A":
          output = xboxA_Operator;
          break;
        case "B":
          output = xboxB_Operator;
          break;
        case "X":
          output = xboxX_Operator;
          break;
        case "Y":
          output = xboxY_Operator;
          break;
        case "LS":
          output = xboxL3_Operator;
          break;
        case "RS":
          output = xboxL3_Operator;
          break;
        case "LB":
          output = xboxLeftBumper_Operator;
          break;
        case "RB":
          output = xboxRightBumper_Operator;
          break;    
        case "START":
          output = xboxStart_Operator;
          break;
        case "BACK":
          output = xboxBack_Operator;
          break;
        default:
          output = xboxBack_Operator;  
      }
    }
    return output;
  }

  public static JoystickAxis xboxAxis(XboxController controller, String axis) {
    JoystickAxis output;
    if(controller == driver) {
      switch (axis.toUpperCase()) {
        case "LT":
          output = xboxLT_Driver;
          break;
        case "RT":
          output = xboxRT_Driver;
          break;
        case "LS-X":
          output = xboxLX_Driver;
          break;
        case "LS-Y":
          output = xboxLY_Driver;
          break;
        case "RS-X":
          output = xboxRX_Driver;
          break;
        case "RS-Y":
          output = xboxRY_Driver;
          break;
        default:
          output = xboxRX_Driver;  
      }
    }
    else
    {
      switch (axis.toUpperCase()) {
        case "LT":
          output = xboxLT_Operator;
          break;
        case "RT":
          output = xboxRT_Operator;
          break;
        case "LS-X":
          output = xboxLX_Operator;
          break;
        case "LS-Y":
          output = xboxLY_Operator;
          break;
        case "RS-X":
          output = xboxRX_Operator;
          break;
        case "RS-Y":
          output = xboxRY_Operator;
          break;
        default:
          output = xboxRX_Operator;    
      }
    }
    return output;
  }

  public static JoystickDPad xboxDPad(XboxController controller, int pov) {
    JoystickDPad output;
    if(controller == driver) {
      switch (pov) {
        case 0:
          output = xboxDPad0_Driver;
          break;
        case 45:
          output = xboxDPad45_Driver;
          break;
        case 90:
          output = xboxDPad90_Driver;
          break;
        case 135:
          output = xboxDPad135_Driver;
          break;
        case 180:
          output = xboxDPad180_Driver;
          break;
        case 225:
          output = xboxDPad225_Driver;
          break;
        case 270:
          output = xboxDPad270_Driver;  
          break;
        case 315:
          output = xboxDPad315_Driver;  
          break;
        default:
          output = xboxDPad315_Driver;  
      }
    }
    else
    {
      switch (pov) {
        case 0:
          output = xboxDPad0_Operator;
          break;
        case 45:
          output = xboxDPad45_Operator;
          break;
        case 90:
          output = xboxDPad90_Operator;
          break;
        case 135:
          output = xboxDPad135_Operator;
          break;
        case 180:
          output = xboxDPad180_Operator;
          break;
        case 225:
          output = xboxDPad225_Operator;
          break;
        case 270:
          output = xboxDPad270_Operator;  
          break;
        case 315:
          output = xboxDPad315_Operator;  
          break;
        default:
          output = xboxDPad315_Operator;  
      }
    }
    return output;
  }
}
  
  
