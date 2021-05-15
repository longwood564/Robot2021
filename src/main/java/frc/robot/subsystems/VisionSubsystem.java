// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.SendableChooser;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

/**
 * Represents the vision subsystem.
 *
 * <p>The vision subsystem encapsulates managing cameras attached to the roboRIO, and interfacing
 * with coprocessors to provide object tracking data from the other camera.
 */
public class VisionSubsystem extends SubsystemBase implements Loggable {
  /**
   * Represents a arrangement of power cells. This is for simulation purposes only - signalling the
   * power cell positions during an actual Galactic Search challenge attempt is prohibited by
   * GSC1-2.
   */
  private enum PowerCellArrangement {
    kARed(new Translation2d(90, 90), new Translation2d(150, 60), new Translation2d(180, 150)),
    kABlue(new Translation2d(180, 30), new Translation2d(210, 120), new Translation2d(270, 90)),
    kBRed(new Translation2d(90, 120), new Translation2d(150, 60), new Translation2d(210, 120)),
    kBBlue(new Translation2d(240, 120), new Translation2d(180, 60), new Translation2d(300, 60));

    public final SimVisionTarget[] value;

    PowerCellArrangement(Translation2d... translations) {
      value = new SimVisionTarget[translations.length];
      for (int i = 0; i < translations.length; ++i)
        value[i] =
            new SimVisionTarget(
                // Convert all of the measurements from inches to meters.
                new Pose2d(translations[i].times(0.0254), new Rotation2d(0)),
                0,
                FieldConstants.kPowerCellDiameter,
                FieldConstants.kPowerCellDiameter);
    }
  }

  // Power Cell Arrangement Chooser
  @Log(name = "Power Cell Arrangement Chooser")
  private SendableChooser<PowerCellArrangement> m_arrangementChooser;

  @Log.CameraStream(
      name = "Camera",
      width = 7,
      height = 6,
      rowIndex = 0,
      columnIndex = 0,
      tabName = "Driver View")
  // USB Camera Class
  UsbCamera m_camera;

  // PhotonLib Camera Class
  private PhotonCamera m_photonCamera = new PhotonCamera(VisionConstants.kCamName);

  // PhotonLib Simulation

  // Shooter Low
  private SimVisionSystem m_photonSimLow;
  // Shooter High
  private SimVisionSystem m_photonSimHigh;

  // Pose Supplier (Used for simulation)
  private final Supplier<Pose2d> m_poseSupplier;

  @Log(name = "Has Targets")
  private boolean m_hasTargets = false;

  // Current distance to target.
  private double m_distanceToTarget;

  // Current pitch to target.
  private double m_yawToTarget;

  /** Initializes the vision subsystem. */
  public VisionSubsystem(Supplier<Pose2d> poseSupplier) {
    m_poseSupplier = poseSupplier;

    // If we're runnning on the real robot, enable the camera.
    if (RobotBase.isReal()) {
      // m_camera = CameraServer.getInstance().startAutomaticCapture();
      // m_camera.setResolution(960, 720);
    } else {
      initializeSimulatedSystems();

      m_arrangementChooser = new SendableChooser<>();
      m_arrangementChooser.setDefaultOption("Path A Red", PowerCellArrangement.kARed);
      m_arrangementChooser.addOption("Path A Blue", PowerCellArrangement.kABlue);
      m_arrangementChooser.addOption("Path B Red", PowerCellArrangement.kBRed);
      m_arrangementChooser.addOption("Path B Blue", PowerCellArrangement.kBBlue);
      // When an option is selected, update the simulation.
      m_arrangementChooser.addListener(
          arrangement -> {
            setSimTargets(arrangement.value);
          });
    }
  }

  /** Initializes the simulated vision systems. */
  private void initializeSimulatedSystems() {
    m_photonSimLow =
        new SimVisionSystem(
            VisionConstants.kCamName,
            VisionConstants.kCamDiagonalFOV,
            VisionConstants.kCamPitchLow,
            VisionConstants.kTransCamToRobot,
            VisionConstants.kCamHeightOffGroundLow,
            VisionConstants.kCamMaxLEDRange,
            VisionConstants.kCamResolutionWidth,
            VisionConstants.kCamResolutionHeight,
            VisionConstants.kMinTargetArea);
    m_photonSimHigh =
        new SimVisionSystem(
            VisionConstants.kCamName,
            VisionConstants.kCamDiagonalFOV,
            VisionConstants.kCamPitchHigh,
            VisionConstants.kTransCamToRobot,
            VisionConstants.kCamHeightOffGroundHigh,
            VisionConstants.kCamMaxLEDRange,
            VisionConstants.kCamResolutionWidth,
            VisionConstants.kCamResolutionHeight,
            VisionConstants.kMinTargetArea);
  }

  /** This method is run periodically. */
  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      m_photonSimLow.processFrame(m_poseSupplier.get());
    }

    // Get the latest pipeline result.
    PhotonPipelineResult pipelineResult = m_photonCamera.getLatestResult();

    // Update the telemetry.
    m_hasTargets = pipelineResult.hasTargets();

    // Update the best target.
    if (m_hasTargets) {
      PhotonTrackedTarget target = pipelineResult.getBestTarget();
      m_yawToTarget = Units.degreesToRadians(target.getYaw());
      m_distanceToTarget =
          PhotonUtils.calculateDistanceToTargetMeters(
              VisionConstants.kCamHeightOffGroundLow,
              FieldConstants.kPowerCellDiameter,
              VisionConstants.kCamPitchLow,
              Units.degreesToRadians(target.getPitch()));
      SmartDashboard.putNumber("PV Target Area", target.getArea());
    } else {
      m_distanceToTarget = 0;
      m_yawToTarget = 0;
      SmartDashboard.putNumber("PV Target Area", 0);
    }
    SmartDashboard.putNumber("PV Distance to Target", m_distanceToTarget);
    SmartDashboard.putNumber("PV Yaw to Target", m_yawToTarget);
  }

  public void setSimTargets(SimVisionTarget[] targets) {
    // Currently, PhotonLib doesn't support clearing the existing simulated targets, so here we
    // reconstruct the systems.
    initializeSimulatedSystems();

    for (SimVisionSystem photonSim : List.of(m_photonSimLow, m_photonSimHigh)) {
      for (SimVisionTarget target : targets) photonSim.addSimVisionTarget(target);
    }
  }

  /**
   * Returns whether targets have been found.
   *
   * @return Whether targets have been found.
   */
  public boolean hasTargets() {
    return m_hasTargets;
  }

  /**
   * Returns the latest distance to the target.
   *
   * @return The latest distance to the target. 0 if a target isn't detected.
   */
  public double getDistanceToTarget() {
    return m_distanceToTarget;
  }

  /**
   * Returns the latest yaw to the target.
   *
   * @return The latest yaw to the target. 0 if a target isn't detected.
   */
  public double getYawToTarget() {
    return m_yawToTarget;
  }
}
