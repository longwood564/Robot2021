// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.GalacticSearchConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommand.Mode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GalacticSearchCommand extends SequentialCommandGroup {

  /** Initializes the command. */
  public GalacticSearchCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubsystem,
      VisionSubsystem visionSubsystem) {
    for (int i = 1; i < 4; ++i)
      addCommands(new SearchForGalactic(drivetrainSubsystem, intakeSubsystem, visionSubsystem, i));
    addCommands(
        new DriveTrajectoryCommand(
            drivetrainSubsystem,
            drivetrainSubsystem.getPose(),
            new ArrayList<Translation2d>(),
            new Pose2d(
                new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(90)),
                new Rotation2d())));
  }

  private class SearchForGalactic extends SequentialCommandGroup {

    /** Initializes the command. */
    public SearchForGalactic(
        DrivetrainSubsystem drivetrainSubsystem,
        IntakeSubsystem intakeSubsystem,
        VisionSubsystem visionSubsystem,
        int n) {
      addCommands(
          // Drive to the power cell.
          new DriveToTargetCommand(
              drivetrainSubsystem, visionSubsystem, GalacticSearchConstants.kStopDistance),
          // Print the informational message.
          new PrintCommand("At target " + n + "!"),
          // Turn the robot around.
          new DriveCommand(
              drivetrainSubsystem,
              Mode.Constant,
              0,
              Mode.Control,
              Units.degreesToRadians(180),
              true),
          // Start the intake mechanism.
          new InstantCommand(() -> intakeSubsystem.startIntake(IntakeConstants.kSpeedIntake)),
          new InstantCommand(() -> intakeSubsystem.startBelt(IntakeConstants.kSpeedBelt)),
          // Back up to the power cell.
          new DriveCommand(
              drivetrainSubsystem,
              Mode.Control,
              -(GalacticSearchConstants.kStopDistance
                  + GalacticSearchConstants.kAdditionalDistance),
              Mode.Constant,
              0),
          // Stop the intake mechanism.
          new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem),
          new InstantCommand(intakeSubsystem::stopBelt, intakeSubsystem));
    }
  }
}
