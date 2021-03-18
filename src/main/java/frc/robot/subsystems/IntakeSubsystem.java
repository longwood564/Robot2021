// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;
import frc.robot.Constants.RoboRIO;

/**
 * Represents the intake subsystem.
 *
 * <p>The intake subsystem encapsulates the motors used to feed balls into the robot, any sensors
 * used to keep track of them, and the motors used to transport the balls through the robot.
 */
public class IntakeSubsystem extends SubsystemBase implements Loggable {
  // Motor Controllers

  @Log(name = "Intake Motor", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
  @Log(
      name = "Intake Motor",
      width = 3,
      height = 1,
      rowIndex = 3,
      columnIndex = 7,
      tabName = "Driver View")
  private final WPI_TalonSRX m_motorIntake = new WPI_TalonSRX(RoboRIO.CAN.kPortMotorIntake);

  @Log(name = "Belt Motor", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
  @Log(
      name = "Belt Motor",
      width = 3,
      height = 1,
      rowIndex = 4,
      columnIndex = 7,
      tabName = "Driver View")
  private final WPI_VictorSPX m_motorBelt = new WPI_VictorSPX(RoboRIO.CAN.kPortMotorBelt);

  // Filters

  // Single-Pole Infinite Impulse Response Low-Pass Filter
  LinearFilter m_lowPassFilter = LinearFilter.singlePoleIIR(0.1, Constants.kRobotLoopPeriod);

  // Moving Average Filter
  LinearFilter m_movingAverageFilter = LinearFilter.movingAverage(5);

  // Single-Pole Infinite Impulse Response High-Pass Filter
  LinearFilter m_highPassFilter = LinearFilter.highPass(0.1, Constants.kRobotLoopPeriod);
  LinearFilter m_highPassFilter2 = LinearFilter.highPass(0.1, Constants.kRobotLoopPeriod);

  // Intake motor current draw.
  @Log private double m_currentDraw = 0;

  // Number of balls stored.
  @Log private int m_numBalls = 0;

  // Whether a ball is currently being taken in.
  @Log private boolean m_takingInBall = false;

  /** This method is run periodically. */
  @Override
  public void periodic() {
    m_currentDraw = m_motorIntake.getSupplyCurrent();
    // m_currentDraw = m_motorIntake.get() * 3;
    SmartDashboard.putNumber("Current Draw", m_currentDraw);
    double lowPass = m_lowPassFilter.calculate(m_currentDraw);
    SmartDashboard.putNumber("Low Pass", lowPass);
    SmartDashboard.putNumber("Moving Average", m_movingAverageFilter.calculate(m_currentDraw));
    SmartDashboard.putNumber("High Pass", m_highPassFilter.calculate(m_currentDraw));
    SmartDashboard.putNumber("Low + High Pass", m_highPassFilter2.calculate(lowPass));
    if (m_currentDraw >= 2) {
      if (!m_takingInBall) {
        System.out.println("Motor is fighting back! Probably taking in a ball.");
        m_takingInBall = true;
        m_numBalls++;
      }
    } else {
      m_takingInBall = false;
    }
  }

  /**
   * Starts the ball intake motor to take in balls.
   *
   * @param speed The speed to set the motor to.
   */
  public void startIntake(double speed) {
    m_motorIntake.set(speed);
  }

  /** Stops the ball intake motor. */
  public void stopIntake() {
    m_motorIntake.set(0);
  }

  /**
   * Starts the belt motor to move the balls forwards or backwards.
   *
   * @param speed The speed to set the motor to.
   */
  public void startBelt(double speed) {
    m_motorBelt.set(speed);
  }

  /** Stops the belt motor. */
  public void stopBelt() {
    m_motorBelt.set(0);
  }

  public boolean isTakingInBall() {
    return m_takingInBall;
  }
}
