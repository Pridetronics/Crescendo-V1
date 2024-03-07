// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.SwerveAutoPaths;
import frc.robot.utils.NoteDepositPosition;
import frc.robot.utils.NotePosition;
import frc.robot.utils.NoteDepositPosition.DepositLocation;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically imCANID this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CameraConstants {
    public static final String kHostName = "robotcamera";
    public static final String kCameraName = "Camera_Module_v1";
    public static final Transform3d kRobotToCamera = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(14.5), 
        0, 
        Units.inchesToMeters(5.5)
      ), 
      new Rotation3d(0, Units.degreesToRadians(-20), 0)
    );

  }

  public static class ClimberConstants {
    public static final int climberLeftMotorID = 15;
    public static final int climberRightMotorID = 16;
    public static final int climberLeftLimitSwitchID = 2;
    public static final int climberRightLimitSwitchID = 1;
  } //End of Class

  public static class ShooterConstants {
    public static final int kShooterMotorCANID = 13; //Our Motor ID
    public static final double kShooterPValue = 0.0002;
    public static final double kShooterIValue = 0.0000005;
    public static final double kShooterDValue = 0.0029;
    public static final int kShooterRPM = 5800; //Our shooter RPM
    public static final int TimeToShootSeconds = 10; //This tells us when we want to stop shooting
    public static final int kMinRPMForIntake = 5000; //Minimum RPM needed for putting a note into the shooter
    public static final int kShootForAmpRPM = 1000; //Setting an initial value for our AMP shooting
    public static final int kMinForAmpRPM = 800; //Setting our minimum AMP shooting value
  } //End of Class

  public static class IntakeConstants {
    public static final int kIntakeMotorCANID = 14; //Our Motor ID
    public static final double kIntakePValue = 0.0001;
    public static final double kIntakeIValue = 0.000001;
    public static final double kIntakeDValue = 0.002;
    public static final int upperSensorChannelID = 0; //Which sensor is the upper sensor
    public static final int lowerSensorChannelID =3; //Which sensor is the lower sensor
    public static final int kIntakeRPM = 7500; //Setting our intake RPM
    public static final int kReverseIntakeRPM = -7500; //Creating a reverse value for exceptions
  } //End of Class

  //Constants for features related to user controller input
  public static class IOConstants {
    //Deadband for the joysticks (for driving inputs so the robot doesnt move when not touching controller)
    public static final double kDeadband = 0.1;
    //Identifier for the controller
    public static final int kDriveJoystickID = 0;

    //Axis for right/left movement
    public static final int kDriveJoystickXAxis = 1;
    //Axis for forward/backward movement
    public static final int kDriveJoystickYAxis = 0;
    //Axis for turning
    public static final int kDriveJoystickTurningAxis = 4;
    //Button ID for robot oriented drive (when holding)
    public static final int kDriveFieldOrientedDriveBtnID = 0;
    //Button ID for reseting the orientation of the robot to the forward direction of the robot
    public static final int kZeroHeadingBtnID = 2;
    //Time it takes before you can press the zero heading button again (seconds)
    public static final double kZeroHeadingDebounceTime = 2;
    //Sets our intake ID
    public static final int kIntakeButtonID = 4;
    //Sets our shooter ID
    public static final int KShooterButtonID = 3;
    //Setting button ID for our amp
    public static final int kAmplifierShooterButtonID = 6;
    //Setting our button ID for reversing our intake
    public static final int kReverseIntakeButtonID = 3;
  }

  //Constants for data related to the wheels of each module (not the module itself)
  public static class WheelConstants {

    //Diameter of the wheel
    public static final double kSwerveWheelDiameterMeters = Units.inchesToMeters(4);

    //Gear ratio of the propultion motor (number of wheel rotations per motor rotation)
    public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    //Gear ratio of the turning motor (number of wheel rotations per motor rotation)
    public static final double kTurningMotorGearRatio = 1 / 12.8;

    //The distance traveled, in meters, per rotation of the wheel
    public static final double kDistancePerWheelRotation = kSwerveWheelDiameterMeters*Math.PI;
    //Just because im lazy
    public static final double k360DegreesToRadians = 2*Math.PI;

    //Power value in the PIDController that controls the wheel direction
    public static final double kPTurning = 0.8;

    
    //Distance between the right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    //DIstance between the front and back wheels
    public static final double kWheelBaseLength = Units.inchesToMeters(23.5);

    //Kinematics system that solves for each wheel's direction based on the given target direction ahd turn velocity
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBaseLength/2, -kTrackWidth/2), //Front Left
      new Translation2d(kWheelBaseLength/2, kTrackWidth/2), //Front Right
      new Translation2d(-kWheelBaseLength/2, -kTrackWidth/2), //Back Left
      new Translation2d(-kWheelBaseLength/2, kTrackWidth/2) //Beck Right
    );

  }

  //Constants for the movement of the robot
  public static class DriveConstants {
    public static final double kFieldWidthMeters = Units.inchesToMeters(653.2);

    //The literal max speed each wheel is allowed to go
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

    /*
    When talking about these acceleration values, 
    these values influence the rate of change in a number in such a way that the number being influenced 
    is multiplied by the max speed of the robot so that the influeced number being 0 means no speed, 1 means 
    100% speed, and -1 mean -100% speed
      -A value of 1 means the 0 to max time is 1 second
      -A value of 3 means the 0 to max time is 0.333333 seconds
      -A value of 0.5 means the 0 to max time is 2 seconds
      -A value 0f 0.2 means the 0 to max time is 5 seconds
    You get the idea, the number is the max change in velocity, as a percent of the robot's full speed
    That means that the number is inversley related t0 the 0 to max time
   */
    public static final double kTeleMaxDriveAccelerationUnitsPerSecond = 5;
    public static final double kTeleMaxTurningAccelerationUnitsPerSecond = 5;

    //Max speed of the robot itself
    public static final double kTeleMaxDriveSpeedMetersPerSecond = 4;
    //Max turning speed of the robot specified in degrees but converted to radians (with the "(Math.PI/180)")
    public static final double kTeleMaxTurningSpeedRadiansPerSecond = 225 * (Math.PI/180);



    //ID of the Can Spark Max the propels the swerve module wheel
    public static final int kFrontLeftDriveMotorCANID = 5;
    //ID of the Can Spark Max that turns the swerve module wheel
    public static final int kFrontLeftTurningMotorCANID = 6;
    //Whether the turning direction of the wheel is reversed
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    //Whether the propultion direction of the wheel is reversed
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    //ID of the CTRE CANCoder for getting the absolute position of the wheel 
      //(the encoder is the silly little device on top of the module that is wedged bwtween the two motors)
    public static final int kFrontLeftDriveAbsoluteEncoderCANID = 1;
    //Whether the CANCoder direction is reversed
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    /* Offset of the absolute encoder relative to the forward direction of the wheel
          My advice for setting this value to prevent a headache experience:
            1. Face all wheels in the forward direction of the robot, being as close to perfect as you can
            2. Open the Phoenix Tuner X, connect laptop to RoboRio via USB cable (Not by ethenet cable)
            3. Go through each encoder on the device list, press refresh to get the data for it, and get the current rotation of the encoder in degrees
              - The data you see when pressing refresh will include the absolute position data (NOT THE ONE CALLED "POSITION"; ITS CALLED "ABSOLUTE POSITION"), you will use the Absolute position (without sensor/magnet offset)
              - The number will be a number between 0 and 1 (NOT A NEGATIVE NUMBER)
              - Multiply that number by 360 to convert it to degrees
            4. Take the number and simply just place it below! it will subtract that number from the encoder to make the wheel's forward direction always face forward on the robot

      (make sure the value's units are in degrees, not rotations)
       Your welcome! -Guy who made this program   
    */
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 75.76;

    //Everything above applies for the below modules
    
    public static final int kFrontRightDriveMotorCANID = 7;
    public static final int kFrontRightTurningMotorCANID = 8;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final int kFrontRightDriveAbsoluteEncoderCANID = 2;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = 72.24;


    public static final int kBackRightDriveMotorCANID = 9;
    public static final int kBackRightTurningMotorCANID = 10;
    public static final boolean kBackRightTurningEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;
    public static final int kBackRightDriveAbsoluteEncoderCANID = 3;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = 101.25;

    public static final int kBackLeftDriveMotorCANID = 11;
    public static final int kBackLeftTurningMotorCANID = 12;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderCANID = 4;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 121.72; 
  }

  //Constants related to the autonomous period
  public static class AutoConstants {
    public static final double kNoteGrabDistanceOvershootMeters = 1;

    //Max number of notes the driver can allow the robot to grab
    public static final int kMaxNumberOfNotesToPickInShuffleboard = 6;
    //Locations for depositing notes
    public static class NoteDepositConstants {
      public static final NoteDepositPosition speakerCenterSide = new NoteDepositPosition(
        new Pose2d(1.46, 5.54, Rotation2d.fromDegrees(180)),
        DepositLocation.kSpeakerCenterSide
      );
      public static final NoteDepositPosition speakerAmpSide = new NoteDepositPosition(
        new Pose2d(0.94, 6.75, Rotation2d.fromDegrees(226)),
        DepositLocation.kSpeakerAmpSide
      );
      public static final NoteDepositPosition speakerSourceSide = new NoteDepositPosition(
        new Pose2d(0.94, 4.31, Rotation2d.fromDegrees(135)),
        DepositLocation.kSpeakerSourceSide
      );
      public static final NoteDepositPosition amplifier = new NoteDepositPosition(
        new Pose2d(1.85, 7.66, Rotation2d.fromDegrees(90)),
        DepositLocation.kAmplifier
      );
    }
    public static class NotePositionConstants {

      public static final NotePosition StageClose = new NotePosition(
        new Translation2d(2.74, 4.11), 
        List.of(
          new Translation2d(2.18, 4.12),
          new Translation2d(2.53, 4.78),
          new Translation2d(2.46, 3.53)
        ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kStageWingNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kStageWingNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kStageWingNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kStageWingNoteToSpeakerSourceSide
          )
        ) 
      );
      public static final NotePosition CenterClose = new NotePosition(
        new Translation2d(2.90, 5.54), 
        List.of(
          new Translation2d(2.12, 5.58),
          new Translation2d(2.48, 6.07),
          new Translation2d(2.52, 5.02)
        ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kCenterWingNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kCenterWingNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kCenterWingNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kCenterWingNoteToSpeakerSourceSide
          )
        ) 
      );
      public static final NotePosition AmpClose = new NotePosition(
        new Translation2d(2.90, 6.99), 
        List.of(
          new Translation2d(2.24, 7.04),
          new Translation2d(2.41, 6.55),
          new Translation2d(2.34, 7.36)
        ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kAmplifierWingNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kAmplifierWingNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kAmplifierWingNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kAmplifierWingNoteToSpeakerSourceSide
          )
        ) 
      );

      public static final NotePosition SourceFirstFieldCenter = new NotePosition(
        new Translation2d(8.29, 0.74), 
        List.of(
          new Translation2d(7.54, 0.74),
          new Translation2d(7.73, 1.24)
        ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kFirstSourceCenterLineNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kFirstSourceCenterLineNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kFirstSourceCenterLineNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kFirstSourceCenterLineNoteToSpeakerSourceSide
          )
        ) 
      );
      public static final NotePosition SourceSecondFieldCenter = new NotePosition(
        new Translation2d(8.26, 2.42), 
        List.of(
          new Translation2d(7.59, 1.93),
          new Translation2d(7.53, 2.40),
          new Translation2d(7.58, 2.81)
        ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kSecondSourceCenterLineNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kSecondSourceCenterLineNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kSecondSourceCenterLineNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kSecondSourceCenterLineNoteToSpeakerSourceSide
          )
        ) 
      );
      public static final NotePosition CenterFieldCenter = new NotePosition(
        new Translation2d(8.27, 4.11), 
        List.of(
          new Translation2d(7.65, 3.51),
          new Translation2d(7.54, 4.13),
          new Translation2d(7.66, 4.51)
        ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kCenterCenterLineNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kCenterCenterLineNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kCenterCenterLineNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kCenterCenterLineNoteToSpeakerSourceSide
          )
        ) 
      );
      public static final NotePosition AmpSecondFieldCenter = new NotePosition(
        new Translation2d(8.28, 5.80), 
          List.of(
            new Translation2d(7.70, 5.12),
            new Translation2d(7.50, 5.73),
            new Translation2d(7.60, 6.25)
          ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kSecondAmpCenterLineNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kSecondAmpCenterLineNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kSecondAmpCenterLineNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kSecondAmpCenterLineNoteToSpeakerSourceSide
          )
        ) 
      );
      public static final NotePosition AmpFirstFieldCenter = new NotePosition(
        new Translation2d(8.28, 7.47), 
        List.of(
          new Translation2d(7.71, 6.81),
          new Translation2d(7.55, 7.44)
        ),
        new HashMap<DepositLocation, List<Translation2d>>(
          Map.of(
            DepositLocation.kAmplifier, SwerveAutoPaths.kFirstAmpCenterLineNoteToAmplifier,
            DepositLocation.kSpeakerAmpSide, SwerveAutoPaths.kFirstAmpCenterLineNoteToSpeakerAmpSide,
            DepositLocation.kSpeakerCenterSide, SwerveAutoPaths.kFirstAmpCenterLineNoteToSpeakerCenterSide,
            DepositLocation.kSpeakerSourceSide, SwerveAutoPaths.kFirstAmpCenterLineNoteToSpeakerSourceSide
          )
        ) 
      );
    }

    //Max speed during autonomous
    public static final double kMaxSpeedMetersPerSecond = 5;
    //Acceleration during autonomous (note its in meters, not units)
    public static final double kMaxAccelerationMetersPerSecond = 6;

    //Max turning speed during autonomous
    public static final double kMaxTurningSpeedRadiansPerSecond = 270 * (Math.PI / 180);
    //Acceleration during autonomous (note its in radians, not units)
    public static final double kMaxTurningAccelerationRadiansPerSecond = 360 * (Math.PI / 180);

    //Power Controllers for the robot to keep it on course
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    //This is for the Profiled PID Controller that controls the robot direction
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
      kMaxTurningSpeedRadiansPerSecond,
      kMaxTurningAccelerationRadiansPerSecond);
  }
  //Data for autonomous trajectories
  public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecond
  ).setKinematics(WheelConstants.kDriveKinematics);

  //Class that can store swerve module data for the swerve module class
  public static class SwerveModuleConstants {
      public final int kDriveMotorCANID;
      public final int kTurningMotorCANID;
      public final int kTurningEncoderID;
      public final double kAbsoluteEncoderOffsetDegrees;
      public final boolean kAbsoluteEncoderReversed;
      public final boolean kDriveEncoderReversed;
      public final boolean kTurningEncoderReversed;

      SwerveModuleConstants(
              int driveMotorCANID, 
              int turningMotorCANID, 
              int CTRETurningEncoderID,
              double absoluteEncoderOffsetDegrees,
              boolean absoluteEncoderReversed,
              boolean driveEncoderReversed,
              boolean turningEncoderReversed
          ) {

          kDriveMotorCANID = driveMotorCANID;
          kTurningMotorCANID = turningMotorCANID;
          kTurningEncoderID = CTRETurningEncoderID;
          kAbsoluteEncoderOffsetDegrees = absoluteEncoderOffsetDegrees;// * (Math.PI/180);
          kAbsoluteEncoderReversed = absoluteEncoderReversed;
          kDriveEncoderReversed = driveEncoderReversed;
          kTurningEncoderReversed = turningEncoderReversed;
          
      }
  }

  //Class that stores swerve module constants fot the swerve module class
  public static class SwerveModuleClasses {

    public SwerveModuleConstants backLeft = new SwerveModuleConstants(
      DriveConstants.kBackLeftDriveMotorCANID, 
      DriveConstants.kBackLeftTurningMotorCANID, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderCANID, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderReversed, 
      DriveConstants.kBackLeftDriveEncoderReversed, 
      DriveConstants.kBackLeftTurningEncoderReversed
    );

    public SwerveModuleConstants backRight = new SwerveModuleConstants(
      DriveConstants.kBackRightDriveMotorCANID, 
      DriveConstants.kBackRightTurningMotorCANID, 
      DriveConstants.kBackRightDriveAbsoluteEncoderCANID, 
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kBackRightDriveAbsoluteEncoderReversed, 
      DriveConstants.kBackRightDriveEncoderReversed, 
      DriveConstants.kBackRightTurningEncoderReversed
    );
    public SwerveModuleConstants frontLeft = new SwerveModuleConstants(
      DriveConstants.kFrontLeftDriveMotorCANID, 
      DriveConstants.kFrontLeftTurningMotorCANID, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderCANID, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed, 
      DriveConstants.kFrontLeftDriveEncoderReversed, 
      DriveConstants.kFrontLeftTurningEncoderReversed
    );
    public SwerveModuleConstants frontRight = new SwerveModuleConstants(
      DriveConstants.kFrontRightDriveMotorCANID, 
      DriveConstants.kFrontRightTurningMotorCANID, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderCANID, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderReversed, 
      DriveConstants.kFrontRightDriveEncoderReversed, 
      DriveConstants.kFrontRightTurningEncoderReversed
    );
  }
}