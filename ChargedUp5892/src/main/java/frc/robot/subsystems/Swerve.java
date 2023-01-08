package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
This class contains the code for the swerve drivetrain of the robot.
It contains fields for the Pigeon2 gyro, SwerveDriveOdometry, and an array of SwerveModule objects.
It also contains a Field2d object used to store the robot's pose.
*/
public class Swerve extends SubsystemBase {
  // Contains the Pigeon2 gyro used to measure the robot's orientation
  private final Pigeon2 gyro;

  // Stores the odometry of the swerve drivetrain
  private SwerveDriveOdometry swerveOdometry;

  // An array of SwerveModule objects for each module on the swerve drivetrain
  private SwerveModule[] mSwerveMods;

  // Stores the robot's pose
  private Field2d field;

  public Swerve() {
    // Initializes the gyro
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    // Initializes the odometry
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

    // Initializes the array of SwerveModule objects
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    // Initializes the Field2d object
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  // Sets the desired state for each SwerveModule
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Calculates the desired state of each module using the swerve kinematics
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // Sets the desired state for each SwerveModule
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  // Sets the desired state for each SwerveModule
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    // Sets the desired state for each SwerveModule
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  // Returns the pose of the robot according to the swerve odometry
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  // Resets the pose of the robot according to the given pose
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
  }

  // Returns the states of each SwerveModule
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  // Zeroes the gyro
  public void zeroGyro() {
    gyro.setYaw(0);
  }

  // Returns the yaw of the robot according to the gyro
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    // Updates the swerve odometry
    swerveOdometry.update(getYaw(), getStates());
    field.setRobotPose(getPose());

    // Puts various data to the smartdashboard
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}