// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
// TODO: Move everything inside here.
public final class Constants {
  public static final class RoboRIO {
    public static final class CAN {
      /** The port of the front left drive motor. */
      public static final int kPortMotorDriveFrontLeft = 1;
      /** The port of the back left drive motor. */
      public static final int kPortMotorDriveBackLeft = 2;
      /** The port of the front right drive motor. */
      public static final int kPortMotorDriveFrontRight = 3;
      /** The port of the back right drive motor. */
      public static final int kPortMotorDriveBackRight = 4;
      /** The port of the intake motor. */
      public static final int kPortMotorIntake = 7;
      /** The port of the left launcher motor. */
      public static final int kPortMotorLauncherLeft = 8;
      /** The port of the conveyor belt. */
      public static final int kPortMotorBelt = 10;
      /**
       * The port of the right launcher motor. On the electronics board, this is #8b, but this
       * couldn't be replicated due to software limitaitons.
       */
      public static final int kPortMotorLauncherRight = 11;
    }

    public static final class DIO {
      /** The ports of the left drive encoder. */
      public static final int[] kPortsEncoderDriveLeft = {0, 1, 2, 3};
      /** The ports of the right drive encoder. */
      public static final int[] kPortsEncoderDriveRight = {4, 5, 6, 7};
    }
  }

  public static final class DrivetrainConstants {
    // TODO: Check these speeds - especially kSpeedFast.
    /** The slow speed to drive at. */
    public static final double kSpeedSlow = 0.5;
    /** The normal speed to drive at. */
    public static final double kSpeedNormal = 0.85;
    /** The fast speed to drive at. */
    public static final double kSpeedFast = 1;
  }
}
