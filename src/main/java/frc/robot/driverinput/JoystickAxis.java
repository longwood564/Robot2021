// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverinput;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} that gets its (on/off) state from an {@link GenericHID} axis.
 *
 * <p>This is based off of the WPILib JoystickButton implementation. Here, rather than being
 * activated by a button, the trigger is activated when the axis exceeds a certain threshold.
 */
public class JoystickAxis extends Button {
  private final GenericHID m_joystick;
  private final int m_axisNumber;
  private final double m_lowerBound;

  /**
   * Creates a joystick axis for triggering commands.
   *
   * @param joystick The GenericHID object that has the axis (e.g. Joystick, KinectStick, etc)
   * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
   * @param lowerBound The lower bound of activation of the trigger.
   */
  public JoystickAxis(GenericHID joystick, int axisNumber, double lowerBound) {
    requireNonNullParam(joystick, "joystick", "JoystickAxis");

    m_joystick = joystick;
    m_axisNumber = axisNumber;
    m_lowerBound = lowerBound;
  }

  /**
   * Creates a joystick axis for triggering commands.
   *
   * @param joystick The GenericHID object that has the axis (e.g. Joystick, KinectStick, etc)
   * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
   */
  public JoystickAxis(GenericHID joystick, int axisNumber) {
    requireNonNullParam(joystick, "joystick", "JoystickAxis");

    m_joystick = joystick;
    m_axisNumber = axisNumber;
    m_lowerBound = 0.5;
  }

  /**
   * Gets the value of the joystick axis.
   *
   * @return The value of the joystick axis
   */
  @Override
  public boolean get() {
    return m_joystick.getRawAxis(m_axisNumber) >= m_lowerBound;
  }
}
