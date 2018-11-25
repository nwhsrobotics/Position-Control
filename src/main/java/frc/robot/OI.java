/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.PosHoldToggle;
import frc.robot.commands.PosHoldUpdateParams;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  // Declare some constants.
  private static final double MAX_POS_DEG = 90.0;       // how far to rotate shaft at full joystick deflection.
  private static final int POS_HOLD_AXIS = 0;           // left-right on left joystick
  private static final int POS_HOLD_TOGGLE_BUTTON = 1;  // A button
  private static final int POS_HOLD_UPDATE_BUTTON = 2;  // B button

  private static final Joystick m_controller = new Joystick(0);
  private static final JoystickButton m_a = new JoystickButton(m_controller, POS_HOLD_TOGGLE_BUTTON);
  private static final JoystickButton m_b = new JoystickButton(m_controller, POS_HOLD_UPDATE_BUTTON);

  public OI() {
    m_a.whenPressed(new PosHoldToggle());        // Toggle position hold mode with A button.
    m_b.whenPressed(new PosHoldUpdateParams());  // Update params when B is pressed.
  }

  // Get position to hold, in degrees
  public double readPositionDeg() {
    double input = m_controller.getRawAxis(POS_HOLD_AXIS);  // get joystick position, in range -1.0 to 1.0
    double holdPos = input * MAX_POS_DEG;     // convert to degrees

    return holdPos;
  }
}
