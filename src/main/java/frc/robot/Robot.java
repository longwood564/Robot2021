// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import io.github.oblarg.oblog.Logger;

/** Initializes the robot, and handles mode changes. */
public class Robot extends TimedRobot {
  // Robot Container Class
  private RobotContainer m_robotContainer;

  // Autonomous Command
  private Command m_autonomousCommand;

  /**
   * This method is run when the robot is first started up, regardless of whether real or simulated.
   */
  @Override
  public void robotInit() {
    // Disable Joystick connection warnings. This is redundant with the call to this method in
    // disabledInit(), but is necessary for supressing all errors. If we only run this method in
    // disabledInit(), it seems that one or two warnings still slip by.
    DriverStation.getInstance().silenceJoystickConnectionWarning(true);

    // Disable LiveWindow telemetry. LiveWindow serves two main purposes:
    //  - Monitoring actuator/sensor values. If you look in the constructors of most WPILib
    //    libraries, you'll see that they automatically register themselves in the LiveWindow
    //    system - how convenient!
    //  - While the robot is in test mode, the LiveWindow widgets allow you to set values of the
    //    hardware from the dashboard (e.g. SmartDashboard, Shuffleboard, Glass).
    // By disabling telemetry, the functionality in the first bullet is no longer active in every
    // mode. When test mode is enabled, this configuration is overriden, the functionality in both
    // bullets is activated.
    LiveWindow.disableAllTelemetry();

    // Create the Driver View tab if it doesn't exist. Do this first so that this will appear before
    // te subsystem tabs.
    Shuffleboard.getTab("Driver View");

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Configure the logger.
    Logger.configureLoggingAndConfig(m_robotContainer, false);
  }

  /** This method is run periodically in any mode. */
  @Override
  public void robotPeriodic() {
    // Run the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.
    CommandScheduler.getInstance().run();

    // Update the logger.
    Logger.updateEntries();
  }

  /** This method is run when the robot is first started up, when being simulated. */
  @Override
  public void simulationInit() {}

  /** This method is run periodically in any mode, when being simulated. */
  @Override
  public void simulationPeriodic() {}

  /** This method is run when the robot enters disabled mode. */
  @Override
  public void disabledInit() {
    // Disable joystick connection warnings.
    DriverStation.getInstance().silenceJoystickConnectionWarning(true);
  }

  /** This method is run periodically in disabled mode. */
  @Override
  public void disabledPeriodic() {}

  /** This method is run when the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This method is run periodically in autonomous mode. */
  @Override
  public void autonomousPeriodic() {}

  /** This method is run when the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    // Enable joystick connection warnings.
    DriverStation.getInstance().silenceJoystickConnectionWarning(true);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This method is run periodically in teleoperated mode. */
  @Override
  public void teleopPeriodic() {}

  /** This method is run when the robot enters test mode. */
  @Override
  public void testInit() {
    // Cancel all running commands.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This method is run periodically in test mode. */
  @Override
  public void testPeriodic() {}
}
