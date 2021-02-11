// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriverStationConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.driverinput.F310Controller;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  @Log private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  @Log private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  @Log private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Commands

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Driver Input

  private final F310Controller m_controllerDrive =
      new F310Controller(DriverStationConstants.kPortControllerDrive);
  private final F310Controller m_controllerManip =
      new F310Controller(DriverStationConstants.kPortControllerManip);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands.

    m_drivetrainSubsystem.setDefaultCommand(
        new RunCommand(
                () ->
                    m_drivetrainSubsystem.arcadeDrive(
                        m_controllerDrive.getY(GenericHID.Hand.kLeft),
                        m_controllerDrive.getX(GenericHID.Hand.kRight)),
                m_drivetrainSubsystem)
            .withName("Arcade Drive"));

    // Configure button to command bindings.
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Command setSpeedNormalCommand =
        new InstantCommand(
                () -> m_drivetrainSubsystem.setSpeed(DrivetrainConstants.kSpeedNormal),
                m_drivetrainSubsystem)
            .withName("Set Normal Speed");
    m_controllerDrive
        .axisLt
        .whenPressed(
            new InstantCommand(
                    () -> m_drivetrainSubsystem.setSpeed(DrivetrainConstants.kSpeedSlow),
                    m_drivetrainSubsystem)
                .withName("Set Slow Speed"))
        .whenReleased(setSpeedNormalCommand);
    m_controllerDrive
        .axisRt
        .whenPressed(
            new InstantCommand(
                    () -> m_drivetrainSubsystem.setSpeed(DrivetrainConstants.kSpeedFast),
                    m_drivetrainSubsystem)
                .withName("Set Fast Speed"))
        .whenReleased(setSpeedNormalCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
