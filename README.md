# Robot2021 ![CI Badge](https://github.com/longwood564/Robot2021/workflows/CI/badge.svg)

The next generation Robot2020.

## Contributing
We welcome contributions from team members of all levels of experience! This section will lay our our expectations for the codebase.

### Style Guide

We use the [Google Java Style guide](https://google.github.io/styleguide/javaguide.html) as a starting point for our style, making the following exceptions:
- Braces do not have to be used where optional ([4.1.1](https://google.github.io/styleguide/javaguide.html#s4.1.1-braces-always-used)).
  - Reason: This is less verbose, and is what our formatter opts for.
  - Exception: If the `if` has braces, the `else if` and `else` should have braces.
- We name our constants with lower camel case, preceeded with a `k`, e.g. `kPortControllerDrive` ([5.2.4](https://google.github.io/styleguide/javaguide.html#s5.2.4-constant-names)).
  - Reason: This is preferred by WPILib and other FRC teams.
- We name out field names with lower camel case, preceeded with an `m_`, e.g. `m_drivetrainSubsystem` ([5.2.5](https://google.github.io/styleguide/javaguide.html#s5.2.5-non-constant-field-names)).
  - Reason: This is preferred by WPILib and other FRC teams.

### Naming Variables
Variables should be named such that they are discoverable to intellisense.
- Bad: `kLeftEncoderPort`.
- Good: `kPortEncoderLeft`.

### Organizing Classes

#### Ordering Members
Classes should have their contents ordered like so:
- Enumerations.
- Member variables.
- Constructor.
- Methods.

#### Adding Comments
Comments added to class members can be categorized into 3 categories:
- Javadoc comments, which describe what a field does.
  - This is required for all classes, and their public members. Parameters `@param` and returns `@return` aren't absolutely required.
- Group comments, which make a new "group" of fields. Every member variable should be in a "group".
  - This is required for all class members.
- General explanatory comments.

To give an example within a class:
```java
// Motor Controllers <THIS IS A GROUP COMMENT>

private final WPI_TalonSRX m_motorFrontLeft =
    new WPI_TalonSRX(RoboRIO.CAN.kPortMotorDriveFrontLeft);  // Notice how members of the same group
private final WPI_TalonSRX m_motorFrontRight =               // don't need newlines between each
    new WPI_TalonSRX(RoboRIO.CAN.kPortMotorDriveFrontRight); // member!
private final WPI_VictorSPX m_motorBackLeft =
    new WPI_VictorSPX(RoboRIO.CAN.kPortMotorDriveBackLeft);
private final WPI_VictorSPX m_motorBackRight =
    new WPI_VictorSPX(RoboRIO.CAN.kPortMotorDriveBackRight);

// Quadrature Encoders <THIS IS A GROUP COMMENT>

/** The left relative encoder. <THIS IS A JAVADOC COMMENT> */
private final Encoder m_encoderRelativeLeft =
    new Encoder(
        RoboRIO.DIO.kPortsEncoderDriveLeft[0],
        RoboRIO.DIO.kPortsEncoderDriveLeft[1],
        RoboRIO.DIO.kPortsEncoderDriveLeft[2]);
// This encoder must be reversed so that values will be consistent with the left drive encoder.
                                                             // ^ This is just a general explanatory
                                                             //   comment.
/** The right relative encoder. <THIS IS A JAVADOC COMMENT> */
private final Encoder m_encoderRelativeRight =
    new Encoder(
        RoboRIO.DIO.kPortsEncoderDriveRight[0],
        RoboRIO.DIO.kPortsEncoderDriveRight[1],
        RoboRIO.DIO.kPortsEncoderDriveRight[2],
        true);

// Gyroscope <THIS IS A GROUP COMMENT>
private final Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
```

#### Writing Javadoc Descriptions
For consistency, the most important takeaway of this section is to **omit phrases like "This class _" or "This method _"**. This helps keep documentation from going on too long, and ensures consistency.

When writing a Javadoc description, if the item *does* something, start it with the verb. For example, here's a method which sets a value:
```java
/**
 * Sets the speed that the robot drives at.
 *
 * @param speed The value will be used as the scaling factor [0..1.0].
 */
public void setSpeed(double speed) {
```
If the item *is* something, just say what it is. For example, here's a class that just contains constants:
```java
/** Constants for the Control Area Network/CAN bus. */
public static final class CAN {
```
Now, subsystems are a bit of a weird middle ground. In a way they *are* the physical robot subsystem they control. For these, we say they *represent* it:
```java
/**
 * Represents the drivetrain subsystem.
 *
 * <p>The drivetrain encapsulates the wheels the robot moves around with, the motors that power
 * them, and the encoders that measure their movement. This subsystem also encapsulates the
 * gyrometer, because it is tightly integrated with the robot driving.
 */
public class DrivetrainSubsystem extends SubsystemBase {
```
Notice how this one has a body paragraph. For items which need further description, these can be added.
