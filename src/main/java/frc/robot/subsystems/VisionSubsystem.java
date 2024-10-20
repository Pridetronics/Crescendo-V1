// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera(CameraConstants.kCameraName);
  private AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
    fieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
    camera, 
    CameraConstants.kRobotToCamera
  );
  private Optional<EstimatedRobotPose> currentRobotPose = Optional.empty();
  private boolean currentlyLookingAtAprilTag = false;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    //Done so the camera and camera settings can be viewed at "<HOSTNAME>.local:5800" on google when tethered
    //            ^^^replace <HOSTNAME> with the constants variable for the hostname
    PortForwarder.add(5800, CameraConstants.kHostName + ".local", 5800);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Optional<EstimatedRobotPose> robotPose = poseEstimator.update();
    if (robotPose.isPresent()) currentRobotPose = robotPose;
    currentlyLookingAtAprilTag = robotPose.isPresent();
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return currentRobotPose;
  }
  
  public boolean lookingAtAprilTag() {
    return currentlyLookingAtAprilTag;
  }
}
