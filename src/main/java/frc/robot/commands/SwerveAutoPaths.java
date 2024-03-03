// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public final class SwerveAutoPaths {

  //Naming system: k<NOTE-LOCATION>To<DEPOSIT-LOCATION>
  //List of positions that are in between the note attack location and the deposit location


  //Amp Wing Note
  public static final List<Translation2d> kAmplifierWingNoteToAmplifier = List.of(

  );
  public static final List<Translation2d> kAmplifierWingNoteToSpeakerAmpSide = List.of(
    new Translation2d(1.20, 6.81)
  );
  public static final List<Translation2d> kAmplifierWingNoteToSpeakerCenterSide = List.of(

  );
  public static final List<Translation2d> kAmplifierWingNoteToSpeakerSourceSide = List.of(
    new Translation2d(1.29, 4.11)
  );

  //Center Wing Note
  public static final List<Translation2d> kCenterWingNoteToAmplifier = List.of(
    new Translation2d(1.76, 6.50)
  );
  public static final List<Translation2d> kCenterWingNoteToSpeakerAmpSide = List.of(
    new Translation2d(1.11, 6.93)
  );
  public static final List<Translation2d> kCenterWingNoteToSpeakerCenterSide = List.of(

  );
  public static final List<Translation2d> kCenterWingNoteToSpeakerSourceSide = List.of(
    new Translation2d(1.20, 3.94)
  );

  //Stage Wing Note
  public static final List<Translation2d> kStageWingNoteToAmplifier = List.of(
    new Translation2d(1.92, 5.25)
  );
  public static final List<Translation2d> kStageWingNoteToSpeakerAmpSide = List.of(
    new Translation2d(1.27, 6.86)
  );
  public static final List<Translation2d> kStageWingNoteToSpeakerCenterSide = List.of(
    new Translation2d(1.66, 5.57)
  );
  public static final List<Translation2d> kStageWingNoteToSpeakerSourceSide = List.of(
    new Translation2d(1.16, 4.06)
  );

  //First Source Center Line Note
  public static final List<Translation2d> kFirstSourceCenterLineNoteToAmplifier = List.of(
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16),
    new Translation2d(1.80, 4.41)
  );
  public static final List<Translation2d> kFirstSourceCenterLineNoteToSpeakerAmpSide = List.of(
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16),
    new Translation2d(1.18, 6.98)
  );
  public static final List<Translation2d> kFirstSourceCenterLineNoteToSpeakerCenterSide = List.of(
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16),
    new Translation2d(1.78, 5.44)
  );
  public static final List<Translation2d> kFirstSourceCenterLineNoteToSpeakerSourceSide = List.of(
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16)
  );

  //Second Source Center Line Note
  public static final List<Translation2d> kSecondSourceCenterLineNoteToAmplifier = List.of(
    new Translation2d(6.68, 1.45),
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16),
    new Translation2d(1.80, 4.41)
  );
  public static final List<Translation2d> kSecondSourceCenterLineNoteToSpeakerAmpSide = List.of(
    new Translation2d(6.68, 1.45),
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16),
    new Translation2d(1.18, 6.98)
  );
  public static final List<Translation2d> kSecondSourceCenterLineNoteToSpeakerCenterSide = List.of(
    new Translation2d(6.68, 1.45),
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16),
    new Translation2d(1.78, 5.44)
  );
  public static final List<Translation2d> kSecondSourceCenterLineNoteToSpeakerSourceSide = List.of(
    new Translation2d(6.68, 1.45),
    new Translation2d(5.18, 1.68),
    new Translation2d(2.27, 3.16)
  );

  //Center Center Line Note
  public static final List<Translation2d> kCenterCenterLineNoteToAmplifier = List.of(
    new Translation2d(6.94, 5.99),
    new Translation2d(5.63, 6.70),
    new Translation2d(3.15, 6.26),
    new Translation2d(2.04, 6.26)
  );
  public static final List<Translation2d> kCenterCenterLineNoteToSpeakerAmpSide = List.of(
    new Translation2d(6.94, 5.99),
    new Translation2d(5.63, 6.70),
    new Translation2d(3.15, 6.26),
    new Translation2d(2.04, 6.26),
    new Translation2d(1.09, 6.95)
  );
  public static final List<Translation2d> kCenterCenterLineNoteToSpeakerCenterSide = List.of(
    new Translation2d(6.94, 5.99),
    new Translation2d(5.63, 6.70),
    new Translation2d(3.15, 6.26),
    new Translation2d(2.04, 6.26)
  );
  public static final List<Translation2d> kCenterCenterLineNoteToSpeakerSourceSide = List.of(
    new Translation2d(6.15, 1.63),
    new Translation2d(3.13, 2.48)
  );

  //Second Amp Center Line Note
  public static final List<Translation2d> kSecondAmpCenterLineNoteToAmplifier = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25)
  );
  public static final List<Translation2d> kSecondAmpCenterLineNoteToSpeakerAmpSide = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25),
    new Translation2d(1.07, 6.94)
  );
  public static final List<Translation2d> kSecondAmpCenterLineNoteToSpeakerCenterSide = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25),
    new Translation2d(1.68, 5.56)
  );
  public static final List<Translation2d> kSecondAmpCenterLineNoteToSpeakerSourceSide = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25),
    new Translation2d(1.33, 4.12)
  );

  //First Amp Center Line Note
  public static final List<Translation2d> kFirstAmpCenterLineNoteToAmplifier = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25)
  );
  public static final List<Translation2d> kFirstAmpCenterLineNoteToSpeakerAmpSide = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25),
    new Translation2d(1.07, 6.94)
  );
  public static final List<Translation2d> kFirstAmpCenterLineNoteToSpeakerCenterSide = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25),
    new Translation2d(1.68, 5.56)
  );
  public static final List<Translation2d> kFirstAmpCenterLineNoteToSpeakerSourceSide = List.of(
    new Translation2d(6.19, 7.04),
    new Translation2d(3.48, 6.20),
    new Translation2d(2.25, 6.25),
    new Translation2d(1.33, 4.12)
  );

  private SwerveAutoPaths() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
