// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Initializes the robot subsystems for teleoperated mode. */
public class TeleopInitCommand extends InstantCommand {
  // Subsystems

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  /** Initializes the command. */
  public TeleopInitCommand(
      DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_shooterSubsystem = shooterSubsystem;

    addRequirements(drivetrainSubsystem, shooterSubsystem);
  }

  /** This method is run periodically while the command is scheduled. */
  @Override
  public void execute() {
    // Set the neutral mode of the drive motors to break, for more responsiveness.
    // m_drivetrainSubsystem.setNeutralMode(NeutralMode.Brake);
    // We don't want to damage the floor of the new gym, so coast in neutral.
    m_drivetrainSubsystem.setNeutralMode(NeutralMode.Coast);

    // Reset the double solenoid.
    m_shooterSubsystem.resetSolenoid();
  }
}
