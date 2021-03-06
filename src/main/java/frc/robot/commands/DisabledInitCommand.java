// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.subsystems.DrivetrainSubsystem;

/** Initializes the robot subsystems for disabled mode. */
public class DisabledInitCommand extends InstantCommand {
  // Subsystems

  private final DrivetrainSubsystem m_drivetrainSubsystem;

  /** Initializes the command. */
  public DisabledInitCommand(DrivetrainSubsystem drivetrainSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
  }

  /** This method is run periodically while the command is scheduled. */
  @Override
  public void execute() {
    // Set the neutral mode of the drive motors to coast.
    m_drivetrainSubsystem.setNeutralMode(NeutralMode.Coast);
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
