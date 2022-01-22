package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {
  // The driver's controller
  public static XboxController driver = new XboxController(Constants.ControlConstants.XBOX_CONTROLLER_DRIVER);
  public static XboxController operator = new XboxController(Constants.ControlConstants.XBOX_CONTROLLER_OPERATOR);

  public static XboxController getXboxController(String controller) {
    if (controller.toUpperCase().equals("DRIVER"))
      return driver;
    return operator;
  }

  public static boolean xboxButton(XboxController controller, String button) {
    boolean output = false;
    switch (button.toUpperCase()) {
      case "A":
        output = controller.getAButton();
        break;
      case "B":
        output = controller.getBButton();
        break;
      case "X":
        output = controller.getXButton();
        break;
      case "Y":
        output = controller.getYButton();
        break;
      case "LS":
        output = controller.getLeftStickButton();
        break;
      case "RS":
        output = controller.getRightStickButton();
        break;
      case "START":
        output = controller.getStartButton();
        break;
      case "BACK":
        output = controller.getBackButton();
        break;
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