package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class PoseEstimator extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final Swerve swerve;

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
      new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),
      new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0)))));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimator(PhotonCamera photonCamera, Swerve swerve) {
    this.photonCamera = photonCamera;
    this.swerve = swerve;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator = new SwerveDrivePoseEstimator(
        Vision.KINEMATICS,
        swerve.getYaw(),
        swerve.getModulePositions(),
        new Pose2d());

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    // Update pose estimator with the best visible target
    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
        var targetPose = targetPoses.get(fiducialId);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        swerve.getYaw(),
        swerve.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        swerve.getYaw(),
        swerve.getModulePositions(),
        newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

}