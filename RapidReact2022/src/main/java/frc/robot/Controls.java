package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  public static double xboxAxis(XboxController controller, String axis) {
    double output = 0;
    switch (axis.toUpperCase()) {
      case "LS-Y":
        output = controller.getLeftY();
        break;
      case "LS-X":
        output = controller.getLeftX();
        break;
      case "RS-Y":
        output = controller.getRightY();
        break;
      case "RS-X":
        output = controller.getRightX();
        break;
      case "LT":
        output = controller.getLeftTriggerAxis();
        break;
      case "RT":
        output = controller.getRightTriggerAxis();
        break;
    }
    return output;
  }

  public static int xboxDPad(XboxController controller) {
    return controller.getPOV();
  }
}
  
  
