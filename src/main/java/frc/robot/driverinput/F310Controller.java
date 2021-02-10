// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) 2021, Oscar Robotics - FRC 832 - FTC 5494
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.driverinput;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Handle input from Logitech F310 controllers connected to the Driver Station.
 *
 * <p>This is based off of the WPILib XboxController implementation. Some of the changes made
 * include:
 *
 * <ul>
 *   <li>Trigger member variables have been added, to facilitate command based programming.
 *       <ul>
 *         <li>Public JoystickButton member variables have been added for each button.
 *         <li>Public POVButton member variables have been added for the 4 primary directions.
 *       </ul>
 *   <li>The methods for checking buttons have been removed.
 *   <li>The enums have been made private, and organized by ID. The IDs themselves are unchanged
 *       from the Xbox controllers.
 *   <li>Button variable/method names now match their names on the controller.
 * </ul>
 *
 * <p>This class was made with help by code from Team 832
 * (https://github.com/oscarrobotics/GrouchLib) and Team 6391
 * (https://github.com/6391-Ursuline-Bearbotics/2021_UARobotics_Infinite_Recharge).
 */
public class F310Controller extends GenericHID {
  /** Represents a digital button on an F310Controller. */
  private enum Button {
    kA(1),
    kB(2),
    kX(3),
    kY(4),
    kLb(5),
    kRb(6),
    kBack(7),
    kStart(8),
    kLs(9),
    kRs(10);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }

  /** Represents an axis on an F310Controller. */
  public enum Axis {
    kLeftX(0),
    kLeftY(1),
    kLt(2),
    kRt(3),
    kRightX(4),
    kRightY(5);

    public final int value;

    Axis(int value) {
      this.value = value;
    }
  }

  // Joystick Buttons

  public final JoystickButton buttonA = new JoystickButton(this, Button.kA.value);
  public final JoystickButton buttonB = new JoystickButton(this, Button.kB.value);
  public final JoystickButton buttonX = new JoystickButton(this, Button.kX.value);
  public final JoystickButton buttonY = new JoystickButton(this, Button.kY.value);
  public final JoystickButton buttonLb = new JoystickButton(this, Button.kLb.value);
  public final JoystickButton buttonRb = new JoystickButton(this, Button.kRb.value);
  public final JoystickButton buttonBack = new JoystickButton(this, Button.kBack.value);
  public final JoystickButton buttonStart = new JoystickButton(this, Button.kStart.value);
  public final JoystickButton buttonLs = new JoystickButton(this, Button.kLs.value);
  public final JoystickButton buttonRs = new JoystickButton(this, Button.kRs.value);

  // Joystick POVs

  public final POVButton povUp = new POVButton(this, 0);
  public final POVButton povRight = new POVButton(this, 90);
  public final POVButton povDown = new POVButton(this, 180);
  public final POVButton povLeft = new POVButton(this, 270);

  /**
   * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
   *
   * @param port The port on the Driver Station that the joystick is plugged into.
   */
  public F310Controller(final int port) {
    super(port);
  }

  /**
   * Get the X axis value of the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The X axis value of the controller.
   */
  @Override
  public double getX(Hand hand) {
    return hand.equals(Hand.kLeft) ? getRawAxis(Axis.kLeftX.value) : getRawAxis(Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The Y axis value of the controller.
   */
  @Override
  public double getY(Hand hand) {
    return hand.equals(Hand.kLeft) ? getRawAxis(Axis.kLeftY.value) : getRawAxis(Axis.kRightY.value);
  }

  /**
   * Get the trigger axis value of the controller.
   *
   * @param hand Side of controller whose value should be returned.
   * @return The trigger axis value of the controller.
   */
  public double getTriggerAxis(Hand hand) {
    return hand.equals(Hand.kLeft) ? getRawAxis(Axis.kLt.value) : getRawAxis(Axis.kRt.value);
  }
}
