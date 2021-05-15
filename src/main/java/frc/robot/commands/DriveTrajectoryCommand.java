// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive. This is based off of the WPILib RamseteCommand
 * implementation. The reason this exists, rather then extending RamseteCommand, is that
 * RamseteCommand can't really be debugged when parts of it are hidden in its own class. Support for
 * not using the PIDController has been removed.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 */
public class DriveTrajectoryCommand extends CommandBase {
  // Drivetrain Subsystem
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  // Kinematics (Translates chassis speeds to and from wheel speeds)
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);

  // Feedforward Controller (Calculates voltage from velocity)
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          DrivetrainConstants.Feedforward.kS,
          DrivetrainConstants.Feedforward.kV,
          DrivetrainConstants.Feedforward.kA);

  // Ramsete Feedback Controller (Follows path)
  private final RamseteController m_followerController =
      new RamseteController(
          DrivetrainConstants.Feedback.kRamseteB, DrivetrainConstants.Feedback.kRamseteZeta);

  // PID Feedback Controllers (Approaches setpoints)

  private final PIDController m_controllerLeft =
      new PIDController(DrivetrainConstants.Feedback.X.Velocity.kP, 0, 0);
  private final PIDController m_controllerRight =
      new PIDController(DrivetrainConstants.Feedback.X.Velocity.kP, 0, 0);

  // Voltage Constraint
  private final DifferentialDriveVoltageConstraint m_voltageConstraint =
      new DifferentialDriveVoltageConstraint(
          m_feedforward, m_kinematics, DrivetrainConstants.kMaxVolts);

  // Trajectory Configuration
  private final TrajectoryConfig m_trajectoryConfig =
      new TrajectoryConfig(
              DrivetrainConstants.kMaxTrajectoryVelocity,
              DrivetrainConstants.kMaxTrajectoryAcceleration)
          .setKinematics(m_kinematics)
          .addConstraint(m_voltageConstraint);

  // Trajectory (Parameterizes the spline by time)
  private Trajectory m_trajectory;

  // Interior trajectory waypoints, if we need to save them until the command is ran.
  private List<Translation2d> m_interiorWaypoints;

  // Ending pose, if we need to save it until the command is ran.
  Pose2d m_end;

  // Timer
  private final Timer m_timer = new Timer();

  // Wheel speeds as of last execute().
  private DifferentialDriveWheelSpeeds m_prevSpeeds;

  // Time as of last execute().
  private double m_prevTime;

  /**
   * Constructs a command that, when executed, will follow the provided trajectory. PID control and
   * feedforward are handled internally, and outputs are scaled -12 to 12 representing units of
   * volts. The robot's current location is used as the starting pose.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param drivetrainSubsystem The drivetrain subsystem.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending pose.
   */
  public DriveTrajectoryCommand(
      DrivetrainSubsystem drivetrainSubsystem, List<Translation2d> interiorWaypoints, Pose2d end) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_interiorWaypoints = interiorWaypoints;
    m_end = end;

    addRequirements(drivetrainSubsystem);
  }

  /**
   * Constructs a command that, when executed, will follow the provided trajectory. PID control and
   * feedforward are handled internally, and outputs are scaled -12 to 12 representing units of
   * volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param drivetrainSubsystem The drivetrain subsystem.
   * @param start The starting pose.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending pose.
   */
  public DriveTrajectoryCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Pose2d start,
      List<Translation2d> interiorWaypoints,
      Pose2d end) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, m_trajectoryConfig);

    addRequirements(drivetrainSubsystem);
  }

  /** This method is run when the command is initially scheduled. */
  @Override
  public void initialize() {
    // If we are using the robot's current position as a starting point, generate the trajectory
    // now.
    if (m_trajectory == null)
      m_trajectory =
          TrajectoryGenerator.generateTrajectory(
              m_drivetrainSubsystem.getPose(), m_interiorWaypoints, m_end, m_trajectoryConfig);

    m_prevTime = -1;
    State initialState = m_trajectory.sample(0);
    m_prevSpeeds =
        m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    m_controllerLeft.reset();
    m_controllerRight.reset();
  }

  /** This method is run repeatedly while the command is scheduled. */
  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    // If the command has just been initialized.
    if (m_prevTime < 0) {
      // Reset the drive motors to 0.
      m_drivetrainSubsystem.tankDriveVolts(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    // Calculate the target chassis speeds for this time in the trajectory, and convert them to
    // wheel speeds.
    DifferentialDriveWheelSpeeds targetWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            m_followerController.calculate(
                m_drivetrainSubsystem.getPose(), m_trajectory.sample(curTime)));
    double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    // Calculate the feedforward for the given velocity setpoint. For the acceleration, calculate
    // the secant from the previous speed to this speed (delta y/delta x=delta v/delta t).
    double leftFeedforward =
        m_feedforward.calculate(
            leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
    double rightFeedforward =
        m_feedforward.calculate(
            rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

    // Get the actual current wheel speeds.
    DifferentialDriveWheelSpeeds wheelSpeeds = m_drivetrainSubsystem.getWheelSpeeds();

    // In addition to the feed forward, add the feedback to further approach the setpoint. The
    // feedforward should get us most of the way there, then we take the current wheel speed
    // measurements, and use the PID controller to adjust for the difference between that and the
    // desired setpoint we got from the Ramsete controller.
    double leftOutput =
        leftFeedforward
            + m_controllerLeft.calculate(wheelSpeeds.leftMetersPerSecond, leftSpeedSetpoint);
    double rightOutput =
        rightFeedforward
            + m_controllerRight.calculate(wheelSpeeds.rightMetersPerSecond, rightSpeedSetpoint);

    // Drive with the calculated motor outputs.
    m_drivetrainSubsystem.tankDriveVolts(leftOutput, rightOutput);

    // Update the speeds and times.
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  /** This method is run when the command ends. */
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();

    // If the command was interrupted, stop wherever we are.
    if (interrupted) m_drivetrainSubsystem.tankDriveVolts(0, 0);
  }

  /** This method is run to check whether the command is finished. */
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
