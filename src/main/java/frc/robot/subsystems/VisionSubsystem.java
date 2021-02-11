// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Represents the vision subsystem.
 *
 * <p>The vision subsystem encapsulates managing cameras attached to the roboRIO, and interfacing
 * with coprocessors to provide object tracking data from the other camera.
 */
public class VisionSubsystem extends SubsystemBase implements Loggable {
  @Log.CameraStream(name = "Camera", width = 7, height = 6, rowIndex = 0, columnIndex = 0)
  @Log.CameraStream(
      name = "Camera",
      width = 7,
      height = 6,
      rowIndex = 0,
      columnIndex = 0,
      tabName = "Driver View")
  // USB Camera Class
  UsbCamera m_camera = CameraServer.getInstance().startAutomaticCapture();

  /** Initializes the vision subsystem. */
  public VisionSubsystem() {
    m_camera.setResolution(960, 720);
  }
}
