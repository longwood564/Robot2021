// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToTargetCommand extends DriveCommand {
  // Subsystems
  private final VisionSubsystem m_visionSubsystem;

  private final double m_goalRange;

  private boolean m_prevHasTarget = false;

  /** Initializes the command. */
  public DriveToTargetCommand(
      DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, double goalRange) {
    super(drivetrainSubsystem, Mode.ControlCustom, 0, Mode.ControlCustom, 0, false);

    m_visionSubsystem = visionSubsystem;
    m_goalRange = goalRange;

    addRequirements(drivetrainSubsystem, visionSubsystem);
  }

  /** This method is run when the command is initially scheduled. */
  @Override
  public void initialize() {
    m_prevHasTarget = false;

    // We have to negate the PID controller outputs in order for them to apply effort in the correct
    // direction, based off of the range and yaw.
    m_xNegateCustomOutput = true;
    m_tNegateCustomOutput = false;

    super.initialize();
  }

  /** This method is run periodically while the command is scheduled. */
  @Override
  public void execute() {
    if (m_visionSubsystem.hasTargets()) {
      // If we are just starting to track a target, reinitialize.
      if (!m_prevHasTarget) super.initialize();
      // Setup the custom parameters.
      m_xMode = Mode.ControlCustom;
      m_xCustomMeasurement = m_visionSubsystem.getDistanceToTarget();
      m_xSetpoint = m_goalRange;
      SmartDashboard.putNumber("PV Goal Distance", m_goalRange);
      m_tMode = Mode.ControlCustom;
      m_tCustomMeasurement = m_visionSubsystem.getYawToTarget();
      m_tSetpoint = 0;
      m_prevHasTarget = true;
    } else {
      // Even though we have constructed the DriveCommand for custom control, when we don't have a
      // tracked target, we have no measurements to go off of, so we temporarily switch to constant
      // motor speeds.
      m_xMode = Mode.Constant;
      m_xSetpoint = 0;
      m_tMode = Mode.Constant;
      m_tSetpoint = 0.3;
      m_prevHasTarget = false;
    }
    super.execute();
  }
}
