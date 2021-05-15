// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.SimpleMotorFeedforward;

/**
 * Drives the robot, closing on the gyroscope and/or the encoders, using feedfoward and feedback.
 */
public class DriveCommand extends CommandBase {
  // Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  /**
   * Represents an mode to be used for an axis. The mode will affect how the setpoint is used.
   *
   * <p>In order to use custom parameters, you must extend this class, and override execute() such
   * that the derived class updates the setpoint <b>and</b> the custom measurement, and then calls
   * this class' execute().
   */
  public enum Mode {
    /**
     * Set a constant motor speed. A rate limiter is used so that the motor speed setpoints will
     * ramp up to this speed, rather than causing a drastic acceleration spike in the beginning.
     */
    Constant,
    /** Control the motor speed with PID, closing on position. */
    Control,
    /** Control the motor speed with PID, using custom parameters. */
    ControlCustom,
    /** Control the motor speed with profiled PID, closing on velocity. */
    ProfiledControl,
    /** Control the motor speed with profiled PID, using custom parameters. */
    ProfiledControlCustom,
  }

  /** The mode for the x-axis. */
  protected Mode m_xMode;
  /**
   * Setpoint for the x-axis. Interpreting this value:
   *
   * <ul>
   *   <li>If Mode.Constant is being used, this is a velocity setpoint in meters per second.
   *   <li>If any other mode is being used, this is a position setpoint in meters.
   * </ul>
   */
  protected double m_xSetpoint;
  /**
   * Custom measurement for controlling x. If custom parameters are being used, this variable must
   * be populated with the custom measurement to use for the PID Controller.
   */
  protected double m_xCustomMeasurement;
  /**
   * Whether to negate the output of the PID controller being used with custom x
   * measurements/setpoints.*
   */
  protected boolean m_xNegateCustomOutput = false;
  /** The initial drivetrain distance. */
  protected double m_xInit;
  /** The rate limiter used for ramping up to a constant linear velocity. */
  private SlewRateLimiter m_xLimiter =
      new SlewRateLimiter(DrivetrainConstants.kMaxTrajectoryAcceleration);

  /** The mode for theta. */
  protected Mode m_tMode;
  /**
   * Setpoint for theta. Interpreting this value:
   *
   * <ul>
   *   <li>If Mode.Constant is being used, this is a velocity setpoint in radians per second.
   *   <li>If any other mode is being used, this is a position setpoint in radians.
   * </ul>
   *
   * This may be made relative to the drivetrain's rotation when the command is ran, depending on if
   * m_tRelative is set.
   */
  protected double m_tSetpoint;
  /**
   * Custom measurement for controlling theta.
   *
   * @see #m_xCustomMeasurement
   */
  protected double m_tCustomMeasurement;
  /**
   * Whether to negate the output of the PID controller being used with custom theta
   * measurements/setpoints.*
   */
  protected boolean m_tNegateCustomOutput = false;
  /**
   * Whether to treat the theta setpoint as relative to the initial robot heading, if a position
   * setpoint.
   */
  protected boolean m_tIsRelative;
  /** The initial drivetrain heading, if using a relative theta setpoint. */
  protected double m_tInit;
  /** The rate limiter used for ramping up to a constant angular velocity. */
  private SlewRateLimiter m_tLimiter =
      new SlewRateLimiter(DrivetrainConstants.kMaxTurnAcceleration);

  // PID controller for the drivetrain's position on the x-axis.
  protected PIDController m_controllerX =
      new PIDController(
          DrivetrainConstants.Feedback.X.Position.kP,
          0,
          DrivetrainConstants.Feedback.X.Position.kD);
  // PID controller for the drivetrain's position on the x-axis, using profiling.
  protected ProfiledPIDController m_controllerXProfiled =
      new ProfiledPIDController(
          DrivetrainConstants.Feedback.X.Position.kP * 100,
          0,
          DrivetrainConstants.Feedback.X.Position.kD * 100,
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.kMaxTrajectoryVelocity,
              DrivetrainConstants.kMaxTrajectoryAcceleration));
  // PID controller for the drivetrain's position on theta.
  protected PIDController m_controllerTheta =
      new PIDController(
          DrivetrainConstants.Feedback.Theta.Position.kP,
          0,
          DrivetrainConstants.Feedback.X.Position.kD);
  // PID controller for the drivetrain's position on theta, using profiling. Currently not
  // aggresive enough to be useful.
  protected ProfiledPIDController m_controllerThetaProfiled =
      new ProfiledPIDController(
          DrivetrainConstants.Feedback.X.Position.kP * 100,
          0,
          DrivetrainConstants.Feedback.X.Position.kD * 100,
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.kMaxTurnVelocity, DrivetrainConstants.kMaxTurnAcceleration));

  // Kinematics (Translates chassis speeds to and from wheel speeds)
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);

  // Feedforward Controller (Calculates voltage from velocity)
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          DrivetrainConstants.Feedforward.kS,
          DrivetrainConstants.Feedforward.kV,
          DrivetrainConstants.Feedforward.kA);

  // PID controller for the left wheel velocity.
  private PIDController m_controllerLeft =
      new PIDController(DrivetrainConstants.Feedback.X.Velocity.kP, 0, 0);
  // PID controller for the right wheel velocity.
  private PIDController m_controllerRight =
      new PIDController(DrivetrainConstants.Feedback.X.Velocity.kP, 0, 0);

  // Timer
  private final Timer m_timer = new Timer();

  // Wheel speeds as of last execute().
  private double m_prevLeftSpeedSetpoint;
  private double m_prevRightSpeedSetpoint;

  // Time as of last execute().
  private double m_prevTime;

  // Whether the command has just started. This is necessary because, the first time execute() is
  // called, we don't do anything, so the position PID controllers will technically be at their
  // setpoints, so isFinished() will want to return true.
  private boolean m_hasJustStarted = false;

  /**
   * Initializes the command with a non-relative angle, but not necessarily an absolute angle - this
   * convenience constructor is more often used for a constant action on theta.
   *
   * @param drivetrainSubsystem The drivetrain subsystem.
   * @param xMode The action to take for the x-axis. @see #m_xMode.
   * @param xSetpoint The setpoint for the x-axis. @see #m_xSetpoint.
   * @param tMode The action to take for the theta. @see #m_zAction.
   * @param tSetpoint The setpoint for the theta. @see #m_zSetpoint.
   */
  public DriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Mode xMode,
      double xSetpoint,
      Mode tMode,
      double tSetpoint) {
    this(drivetrainSubsystem, xMode, xSetpoint, tMode, tSetpoint, false);
  }
  /**
   * Initializes the command.
   *
   * @param drivetrainSubsystem The drivetrain subsystem.
   * @param xMode The action to take for the x-axis. @see #m_xMode.
   * @param xSetpoint The setpoint for the x-axis. @see #m_xSetpoint.
   * @param tMode The action to take for the theta. @see #m_zAction.
   * @param tSetpoint The setpoint for the theta. @see #m_zSetpoint.
   * @param tRelative Whether to treat the theta setpoint as relative @see #m_tRelative.
   */
  public DriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Mode xMode,
      double xSetpoint,
      Mode tMode,
      double tSetpoint,
      boolean tRelative) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_xSetpoint = xSetpoint;
    m_xMode = xMode;
    m_tSetpoint = tSetpoint;
    m_tMode = tMode;
    m_tIsRelative = tRelative;

    // Configure all of the controllers, even if we might not be using them, in case a derived class
    // wants to do something like switch actions from constant to control.

    m_controllerX.setTolerance(Units.inchesToMeters(1), Units.inchesToMeters(1));
    m_controllerTheta.enableContinuousInput(-Math.PI, Math.PI);
    m_controllerTheta.setTolerance(Units.degreesToRadians(1));
    // Set the controller to be continuous (because it is an angle controller).
    m_controllerThetaProfiled.enableContinuousInput(-Math.PI, Math.PI);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    m_controllerThetaProfiled.setTolerance(Units.degreesToRadians(1));

    // For convenience, if the P gain is being tuned.
    SmartDashboard.putNumber("P Gain (FOR TUNING PURPOSES ONLY)", 0);
    SmartDashboard.putNumber("D Gain (FOR TUNING PURPOSES ONLY)", 0);

    addRequirements(drivetrainSubsystem);
  }

  /** This method is run when the command is initially scheduled. */
  @Override
  public void initialize() {
    // Initialize the appropriate PID controllers, if any.

    m_xInit = m_drivetrainSubsystem.getDistance();
    // TODO: If x-axis profiling is implemented, make sure it gets reset here.
    switch (m_xMode) {
      case Constant:
        m_xLimiter.reset(m_drivetrainSubsystem.getSpeed());
        break;
      case Control:
      case ControlCustom:
        m_controllerX.reset();
        break;
      case ProfiledControl:
      case ProfiledControlCustom:
        m_controllerXProfiled.reset(m_xInit, m_drivetrainSubsystem.getSpeed());
        break;
    }

    // Initialize this even if we aren't using a relative theta setpoint right now, just in case a
    // derived command needs it.
    m_tInit = m_drivetrainSubsystem.getHeading();
    switch (m_tMode) {
      case Constant:
        m_tLimiter.reset(m_drivetrainSubsystem.getTurnRate());
        break;
      case Control:
      case ControlCustom:
        m_controllerTheta.reset();
        break;
      case ProfiledControl:
      case ProfiledControlCustom:
        m_controllerThetaProfiled.reset(m_tInit, m_drivetrainSubsystem.getTurnRate());
        break;
    }

    // If you would like to tune a controller, do so here. Don't leave this uncommented though!
    // m_controllerTheta.setP(SmartDashboard.getNumber("P Gain (FOR TUNING PURPOSES ONLY)", 0));
    // m_controllerTheta.setD(SmartDashboard.getNumber("D Gain (FOR TUNING PURPOSES ONLY)", 0));

    m_hasJustStarted = true;
    m_prevLeftSpeedSetpoint = m_prevRightSpeedSetpoint = 0;
    m_prevTime = -1;
    m_timer.reset();
    m_timer.start();
  }

  /** This method is run periodically while the command is scheduled. */
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
    } else {
      m_hasJustStarted = false;
    }

    double xSpeedSetpoint = 0;

    switch (m_xMode) {
      case Constant:
        xSpeedSetpoint = m_xLimiter.calculate(m_xSetpoint);
        break;
      case Control:
        xSpeedSetpoint =
            m_controllerX.calculate(m_drivetrainSubsystem.getDistance(), m_xInit + m_xSetpoint);
        SmartDashboard.putNumber("x Error", m_controllerX.getPositionError());
        break;
      case ControlCustom:
        xSpeedSetpoint =
            (m_xNegateCustomOutput ? -1.0 : 1.0)
                * m_controllerX.calculate(m_xCustomMeasurement, m_xSetpoint);
        SmartDashboard.putNumber("x Error", m_controllerX.getPositionError());
        break;
      case ProfiledControl:
        xSpeedSetpoint =
            m_controllerXProfiled.calculate(
                m_drivetrainSubsystem.getDistance(), m_xInit + m_xSetpoint);
        SmartDashboard.putNumber("x Error", m_controllerXProfiled.getPositionError());
        break;
      case ProfiledControlCustom:
        xSpeedSetpoint =
            (m_xNegateCustomOutput ? -1.0 : 1.0)
                * m_controllerXProfiled.calculate(m_xCustomMeasurement, m_tSetpoint);
        SmartDashboard.putNumber("x Error", m_controllerXProfiled.getPositionError());
        break;
    }
    SmartDashboard.putNumber("x Speed Setpoint", xSpeedSetpoint);

    double tSpeedSetpoint = 0;

    // Note that this may still just be a constant rotation speed.
    double tSetpointAdjusted = m_tSetpoint + (m_tIsRelative ? m_tInit : 0);
    switch (m_tMode) {
      case Constant:
        tSpeedSetpoint = m_tLimiter.calculate(m_tSetpoint);
        break;
      case Control:
        tSpeedSetpoint =
            m_controllerTheta.calculate(m_drivetrainSubsystem.getHeading(), tSetpointAdjusted);
        SmartDashboard.putNumber("t Error", m_controllerTheta.getPositionError());
        break;
      case ControlCustom:
        tSpeedSetpoint =
            m_tNegateCustomOutput
                ? -1.0
                : 1.0 * m_controllerTheta.calculate(m_tCustomMeasurement, m_tSetpoint);
        SmartDashboard.putNumber("t Error", m_controllerTheta.getPositionError());
        break;
      case ProfiledControl:
        tSpeedSetpoint =
            m_controllerThetaProfiled.calculate(
                m_drivetrainSubsystem.getHeading(), tSetpointAdjusted);
        SmartDashboard.putNumber("t Error", m_controllerThetaProfiled.getPositionError());
        break;
      case ProfiledControlCustom:
        tSpeedSetpoint =
            m_tNegateCustomOutput
                ? -1.0
                : 1.0 * m_controllerThetaProfiled.calculate(m_tCustomMeasurement, m_tSetpoint);
        SmartDashboard.putNumber("t Error", m_controllerThetaProfiled.getPositionError());
        break;
    }
    SmartDashboard.putNumber("t Speed Setpoint", tSpeedSetpoint);

    DifferentialDriveWheelSpeeds targetWheelSpeeds =
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeedSetpoint, 0, tSpeedSetpoint));
    double leftSpeedSetpoint =
        Math.copySign(
            Math.min(
                DrivetrainConstants.kMaxTrajectoryVelocity,
                Math.abs(targetWheelSpeeds.leftMetersPerSecond)),
            targetWheelSpeeds.leftMetersPerSecond);
    double rightSpeedSetpoint =
        Math.copySign(
            Math.min(
                DrivetrainConstants.kMaxTrajectoryVelocity,
                Math.abs(targetWheelSpeeds.rightMetersPerSecond)),
            targetWheelSpeeds.rightMetersPerSecond);

    SmartDashboard.putNumber("Left Speed Setpoint", targetWheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed Setpoint", targetWheelSpeeds.rightMetersPerSecond);

    // Calculate the feedforward for the given velocity setpoint. For the acceleration, calculate
    // the secant from the previous speed to this speed (Δy/Δx=Δv/Δt).
    double leftFeedforward =
        m_feedforward.calculate(m_prevLeftSpeedSetpoint, leftSpeedSetpoint, dt);
    double rightFeedforward =
        m_feedforward.calculate(m_prevRightSpeedSetpoint, rightSpeedSetpoint, dt);

    SmartDashboard.putNumber("Left Feedforward", leftFeedforward);
    SmartDashboard.putNumber("Right Feedforward", rightFeedforward);

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

    SmartDashboard.putNumber("Left Speed", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed", wheelSpeeds.rightMetersPerSecond);

    SmartDashboard.putNumber("Left Output", leftOutput);
    SmartDashboard.putNumber("Right Output", rightOutput);

    // Drive with the calculated motor outputs.
    m_drivetrainSubsystem.tankDriveVolts(leftOutput, rightOutput);

    // Update the speeds and times.
    m_prevLeftSpeedSetpoint = leftSpeedSetpoint;
    m_prevRightSpeedSetpoint = rightSpeedSetpoint;
    m_prevTime = curTime;
  }

  /** This method is run when the command ends. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) m_drivetrainSubsystem.tankDriveVolts(0, 0);

    SmartDashboard.putNumber("x Error", 0);
    SmartDashboard.putNumber("t Error", 0);
    SmartDashboard.putNumber("x Speed Setpoint", 0);
    SmartDashboard.putNumber("t Speed Setpoint", 0);
    SmartDashboard.putNumber("Left Speed Setpoint", 0);
    SmartDashboard.putNumber("Right Speed Setpoint", 0);
    SmartDashboard.putNumber("Left Feedforward", 0);
    SmartDashboard.putNumber("Right Feedforward", 0);
    SmartDashboard.putNumber("Left Speed", 0);
    SmartDashboard.putNumber("Right Speed", 0);
    SmartDashboard.putNumber("Left Output", 0);
    SmartDashboard.putNumber("Right Output", 0);
  }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  @Override
  public boolean isFinished() {
    // While tuning PID constants, you may want to make this command go on forever:
    // if (true) return false;
    if (m_xMode == Mode.Constant && m_tMode == Mode.Constant) return false;
    if (m_hasJustStarted) return false;

    boolean xFinished = false;
    switch (m_xMode) {
      case Constant:
        xFinished = true;
        break;
      case Control:
      case ControlCustom:
        xFinished = m_controllerX.atSetpoint();
        break;
      case ProfiledControl:
      case ProfiledControlCustom:
        xFinished = m_controllerXProfiled.atGoal();
        break;
    }
    boolean tFinished = false;
    switch (m_tMode) {
      case Constant:
        tFinished = true;
        break;
      case Control:
      case ControlCustom:
        tFinished = m_controllerTheta.atSetpoint();
        break;
      case ProfiledControl:
      case ProfiledControlCustom:
        tFinished = m_controllerThetaProfiled.atGoal();
        break;
    }
    return xFinished && tFinished;
  }
}
