package frc.robot.Utilities.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class JoystickDPad extends DPad{
    private final GenericHID m_joystick;
  private final int m_dpadangle;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickDPad(GenericHID joystick, int dpadangle) {

    m_joystick = joystick;
    m_dpadangle = dpadangle;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  public int getPOV() {
    return m_joystick.getPOV();
  }

  @Override
  public boolean get(){
    return (getPOV()==m_dpadangle);
  }
}
