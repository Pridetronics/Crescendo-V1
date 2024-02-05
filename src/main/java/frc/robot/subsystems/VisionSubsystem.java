// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class VisionSubsystem extends SubsystemBase {
  SwerveSubsystem m_SwerveSybsystem;
  private PhotonCamera camera = new PhotonCamera("photonvision");
  private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
    fieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, CameraConstants.kRobotToCamera
  );
  Pose3d lastRobotPose = new Pose3d();
  /** Creates a new PhotonVision. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    m_SwerveSybsystem = swerveSubsystem;
    //Done so the camera/settings can be viewed at photonvision.local:5800 on google when tethered
    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> robotPose = poseEstimator.update();
    if (robotPose.isPresent()) {
      Pose3d currentPose = robotPose.get().estimatedPose;
      lastRobotPose = currentPose;
      m_SwerveSybsystem.resetOdometry(currentPose.toPose2d());
    } else lastRobotPose = null;
  }

  public Pose3d getCurrentRobotPose() {
    return lastRobotPose;
  }
}
