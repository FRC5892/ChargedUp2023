package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;
    public static final int pigeonID = 13;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Autonomous Speeds */
    public static final double balanceSpeed = 0.45;
    public static final double speedyBalanceSpeed = 0.7;
    public static final double speedyBackup = 0.3;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.5);
    public static final double wheelBase = Units.inchesToMeters(20.5);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 2;
    public static final double closedLoopRamp = 0.0; // seem to be useless

    public static final double driveGearRatio = (5.14 / 1.0); // 6.75:1 for L2

    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1 for all L's

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 50;

    /* TODO: test Angle Motor PID Values (these are default, may tune if needed) */
    public static final double angleKP = 0.02;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.000;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0; // TODO: Tune after charactarization
    public static final double driveKI = 0.0; // leave
    public static final double driveKD = 0.0; // leave
    public static final double driveKFF = 0.0; // leave

    /* TODO: Charactarize drivetrain Drive Motor Characterization Values */
    public static final double driveKS = 0.11979;
    public static final double driveKV = 2.3823;
    public static final double driveKA = 0.30034;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* TODO: set Swerve Profiling Values */
    // private static final double SPEED_MULTIPLIER = 0.2;
    public static final double maxSpeed = 5.5; // meters per second
    public static final double maxAngularVelocity = 5; // TODO: Tune

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    // public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 12;

      public static final boolean driveInvert = false;
      public static final double offsetDegree = -133+180;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree + 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final boolean driveInvert = false;
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 11;
      public static final double offsetDegree = -23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree - 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final boolean driveInvert = true;
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 10;
      public static final double offsetDegree = 158;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree + 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final boolean driveInvert = true;
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 9;
      public static final double offsetDegree = 40;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offsetDegree);
      public static final Rotation2d balanceOffset = Rotation2d.fromDegrees(offsetDegree - 45);
      public static final double conversionFactor = 0.060509807;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveInvert, driveMotorID,
          angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.25;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.8;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
  }

  public static final class VisionConstants {
    public static final String USB_CAMERA_NAME = "photonvision";
    public static final double TRACKED_TAG_ROATION_KP = 0;
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(-0.3425, 0.0, -0.233),
        new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
    public static final int TAG_TO_CHASE = 1;
    public static double aimingSpeed;

  }

  // public static final int[] GROUND_INTAKE_PORTS = {0, 0};
  public static final int[] GROUND_INTAKE_SOLENOID_PORTS = { 0, 1, 2, 3, 4, 5 };

  /* LED Ports */
  public static final class LEDConstants {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 105;
  }

}
