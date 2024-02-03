// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera = new PhotonCamera("photonvision");
  private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private Pose3d lastFieldLayoutPosition = new Pose3d();

  /** Creates a new PhotonVision. */
  public VisionSubsystem() {
    //Done so the camera/settings can be viewed at photonvision.local:5800 on google when tethered
    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }

  public Pose3d getFieldPosition() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Optional<Pose3d> targetFieldPosition = fieldLayout.getTagPose(target.getFiducialId());
      if (!targetFieldPosition.isPresent()) return lastFieldLayoutPosition;
        Pose3d fieldPosition = PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(),
        targetFieldPosition.get(), 
        CameraConstants.kCameraToRobot
        );
        lastFieldLayoutPosition = fieldPosition;
    }

    return lastFieldLayoutPosition;
  }

  public Pose3d getRobotToAprilTag() {
    //TODO
    return new Pose3d();
  }
}
