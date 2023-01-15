package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

/*
This is a class for the swerve drive system on the robot. It utilizes a navX gyro to measure the angle of the robot and a SwerveDriveOdometry to measure the position of the robot. There are four SwerveModule objects, each of which is responsible for the individual swerve module. The class also holds a Field2d object which is used for the robot's position with respect to the field.

The drive() method is used to set the desired speed and angle for the robot. The user can decide if they want the desired rotation and speed to be relative to the field or the robot. The setModuleStates() and setModuleRotation() methods are used to set the desired states of each swerve module. The getPose() method returns the pose of the robot in meters. The resetOdometry() method resets the odometry of the robot to the given pose. The resetToAbsolute() method resets all of the swerve modules to the absolute position. The getStates() and getModulePositions() methods return the current states and positions of each swerve module. The zeroGyro() method sets the yaw of the robot to 0. The getYaw() method returns the yaw of the robot.

In the periodic() method, the robot's odometry is updated, and the yaw of the robot is put to the SmartDashboard. The states and positions of each swerve module is also put to the SmartDashboard.
*/

public class Swerve extends SubsystemBase {
  private AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    gyro = new AHRS(SPI.Port.kMXP);
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    field = new Field2d();
    SmartDashboard.putData(field);
  }

  /**
   * Sets the desired speed and angle for the robot. The desired rotation and speed can be relative to the field or the robot.
   * @param translation The desired translation.
   * @param rotation The desired rotation.
   * @param fieldRelative Whether the desired rotation and speed should be relative to the field or the robot.
   * @param isOpenLoop Whether the desired speed should be open loop or closed loop.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  /**
   * Sets the desired states for each SwerveModule.
   * @param desiredStates The desired states for each SwerveModule.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * Sets the desired rotation for each SwerveModule.
   * @param rotation The desired rotation.
   */
  public void setModuleRotation(Rotation2d rotation) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(new SwerveModuleState(0, rotation), false);
    }
  }

  /**
   * Returns the pose of the robot in meters.
   * @return The pose of the robot in meters.
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Returns the Field2d object.
   * @return The Field2d object.
   */
  public Field2d getField() {
    return field;
  }

  /**
   * Resets the odometry of the robot to the given pose.
   * @param pose The pose to reset the odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Resets each SwerveModule to the absolute position.
   */
  public void resetToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Returns the current states of each SwerveModule.
   * @return The current states of each SwerveModule.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }
  /**
   * Returns the current positions of each SwerveModule.
   * @return The current positions of each SwerveModule.
   */
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  /**
   * Sets the yaw of the robot to 0.
   */
  public void zeroGyro() {
    gyro.zeroYaw();
  }

  /**
   * Returns the yaw of the robot.
   * @return The yaw of the robot.
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }
  

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Pigeon2 Yaw", gyro.getYaw());

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
