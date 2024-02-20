// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class VisionSubsystem extends SubsystemBase {
  SwerveSubsystem m_SwerveSybsystem;
  private PhotonCamera camera = new PhotonCamera(NetworkTableInstance.getDefault(), "robotcamera");
  private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
    fieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
    camera, 
    CameraConstants.kRobotToCamera
  );
  Optional<Pose3d> lastRobotPose = Optional.empty();
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    m_SwerveSybsystem = swerveSubsystem;
    //Done so the camera and camera settings can be viewed at robotcamera.local:5800 on google when tethered
    PortForwarder.add(5800, "robotcamera.local", 5800);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //if (!camera.isConnected()) return;
    Optional<EstimatedRobotPose> robotPose = poseEstimator.update();
    if (robotPose.isPresent()) {
      System.out.println("IS PRESENT");
      Pose3d currentPose = robotPose.get().estimatedPose;
      lastRobotPose = Optional.of(currentPose);
    } else {
      lastRobotPose = Optional.empty();
    };
  }

  public Optional<Pose2d> getLastRobotFieldPosition() {
    if (lastRobotPose.isPresent()) {
      return Optional.of(lastRobotPose.get().toPose2d());
    } else {
      return Optional.empty();
    }
  }
}
