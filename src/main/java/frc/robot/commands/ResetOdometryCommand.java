// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetOdometryCommand extends InstantCommand {
  // Subsystems

  private final DrivetrainSubsystem m_drivetrainSubsystem;

  /** Initializes the command. */
  public ResetOdometryCommand(DrivetrainSubsystem drivetrainSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  /** This method is run periodically while the command is scheduled. */
  @Override
  public void execute() {
    m_drivetrainSubsystem.resetOdometry();
  }

  /**
   * Whether the given command should run when the robot is disabled.
   *
   * @return Whether the command should run when the robot is disabled.
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
