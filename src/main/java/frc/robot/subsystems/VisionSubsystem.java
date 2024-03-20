// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera(CameraConstants.kCameraName);
  private AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(
    List.of(
      new AprilTag( //DONE
        1, 
        new Pose3d(
          Units.inchesToMeters(593.68 + CameraConstants.aprilTagXShiftInches - 24), 
          Units.inchesToMeters(9.68 + CameraConstants.aprilTagYShiftInches + 12), 
          Units.inchesToMeters(53.38), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(120)
          )
        )
      ),
      new AprilTag( //DONE
        2, 
        new Pose3d(
          Units.inchesToMeters(637.21 + CameraConstants.aprilTagXShiftInches - 24), 
          Units.inchesToMeters(34.79 + CameraConstants.aprilTagYShiftInches + 12), 
          Units.inchesToMeters(53.38) , 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(120)
          )
        )
      ),
      new AprilTag(//DONE
        3, 
        new Pose3d(
          Units.inchesToMeters(652.73 + CameraConstants.aprilTagXShiftInches - 48), 
          Units.inchesToMeters(196.17 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(57.13), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(180)
          )
        )
      ),
      new AprilTag(//DONE
        4, 
        new Pose3d(
          Units.inchesToMeters(652.73 + CameraConstants.aprilTagXShiftInches - 48), 
          Units.inchesToMeters(218.42 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(57.13), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(180)
          )
        )
      ),
      new AprilTag(//DONE
        5, 
        new Pose3d(
          Units.inchesToMeters(578.77 + CameraConstants.aprilTagXShiftInches - 12), 
          Units.inchesToMeters(323.00 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(53.38), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(270)
          )
        )
      ),
      new AprilTag(//DONE
        6, 
        new Pose3d(
          Units.inchesToMeters(72.5 + CameraConstants.aprilTagXShiftInches), 
          Units.inchesToMeters(323.00 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(53.38), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(270)
          )
        )
      ),
      new AprilTag(//DONE
        7, 
        new Pose3d(
          Units.inchesToMeters(-1.50 + CameraConstants.aprilTagXShiftInches), 
          Units.inchesToMeters(218.42 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(57.13), 
          new Rotation3d(
            0,
            0, 
            0
          )
        )
      ),
      new AprilTag(//DONE
        8, 
        new Pose3d(
          Units.inchesToMeters(-1.50 + CameraConstants.aprilTagXShiftInches), 
          Units.inchesToMeters(196.17 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(57.13), 
          new Rotation3d(
            0,
            0, 
            0
          )
        )
      ),
      new AprilTag(//DONE
        9, 
        new Pose3d(
          Units.inchesToMeters(14.02 + CameraConstants.aprilTagXShiftInches), 
          Units.inchesToMeters(34.79 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(53.38), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(60)
          )
        )
      ),
      new AprilTag(//DONE
        10, 
        new Pose3d(
          Units.inchesToMeters(57.54 + CameraConstants.aprilTagXShiftInches - 12), 
          Units.inchesToMeters(9.68 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(53.38), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(60)
          )
        )
      ),
      new AprilTag(//DONE
        11, 
        new Pose3d(
          Units.inchesToMeters(468.69 + CameraConstants.aprilTagXShiftInches - 24), 
          Units.inchesToMeters(146.19 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(52.00), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(300)
          )
        )
      ),
      new AprilTag(//DONE
        12, 
        new Pose3d(
          Units.inchesToMeters(468.69 + CameraConstants.aprilTagXShiftInches - 36), 
          Units.inchesToMeters(177.10 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(52.00), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(60)
          )
        )
      ),
      new AprilTag(//DONE
        13, 
        new Pose3d(
          Units.inchesToMeters(441.74 + CameraConstants.aprilTagXShiftInches - 12), 
          Units.inchesToMeters(161.62 + CameraConstants.aprilTagYShiftInches + 12), 
          Units.inchesToMeters(52.00), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(180)
          )
        )
      ),
      new AprilTag(//DONE
        14, 
        new Pose3d(
          Units.inchesToMeters(209.48 + CameraConstants.aprilTagXShiftInches), 
          Units.inchesToMeters(161.62 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(52.00), 
          new Rotation3d(
            0,
            0, 
            0
          )
        )
      ),
      new AprilTag(//DONE
        15, 
        new Pose3d(
          Units.inchesToMeters(182.73 + CameraConstants.aprilTagXShiftInches), 
          Units.inchesToMeters(177.10 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(52.00), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(120)
          )
        )
      ),
      new AprilTag(//DONE
        16, 
        new Pose3d(
          Units.inchesToMeters(182.73 + CameraConstants.aprilTagXShiftInches - 16), 
          Units.inchesToMeters(146.19 + CameraConstants.aprilTagYShiftInches), 
          Units.inchesToMeters(52.00), 
          new Rotation3d(
            0,
            0, 
            Units.degreesToRadians(240)
          )
        )
      )
    ), 
    Units.inchesToMeters(653.2),
    Units.inchesToMeters(323.28)
  );
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
