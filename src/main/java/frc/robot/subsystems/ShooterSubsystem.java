// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.RoboRIO;

/**
 * Represents the shooter subsystem.
 *
 * <p>The shooter subsystem encapsulates the motors used to shoot balls out of the cannon.
 */
public class ShooterSubsystem extends SubsystemBase implements Loggable {
  // Motor Controllers

  @Log(name = "Shooter Motor", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
  @Log(
      name = "Shooter Motor",
      width = 3,
      height = 1,
      rowIndex = 5,
      columnIndex = 7,
      tabName = "Driver View")
  private final WPI_TalonSRX m_motorShooterLeft =
      new WPI_TalonSRX(RoboRIO.CAN.kPortMotorLauncherLeft);

  private final WPI_TalonSRX m_motorShooterRight =
      new WPI_TalonSRX(RoboRIO.CAN.kPortMotorLauncherRight);

  // Solenoids

  @Log
  private final DoubleSolenoid m_doubleSolenoidShooter =
      new DoubleSolenoid(
          RoboRIO.CAN.kPortsDoubleSolenoidLauncherCannon[0],
          RoboRIO.CAN.kPortsDoubleSolenoidLauncherCannon[1]);

  /** Initializes the shooter subsystem. */
  public ShooterSubsystem() {
    // Make the right motor follow the left.
    m_motorShooterRight.follow(m_motorShooterLeft);
  }

  /**
   * Resets the position of the double solenoid. This is nice to have because exiting test mode
   * resets the solenoid to kOff, which breaks toggle().
   */
  public void resetSolenoid() {
    m_doubleSolenoidShooter.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Starts the shooter motor to shoot balls.
   *
   * @param speed The speed to set the motor to.
   */
  public void startShooting(double speed) {
    m_motorShooterLeft.set(speed);
  }

  /** Stops the shooter motor. */
  public void stopShooting() {
    m_motorShooterLeft.set(0);
  }

  /** Toggles the double solenoids to raise or retract the shooter. */
  public void toggleSolenoid() {
    m_doubleSolenoidShooter.toggle();
  }
}
