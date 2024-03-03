// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants.NoteDepositConstants;

/** Add your docs here. */
public class NoteDepositPosition {
    public enum DepositLocation {
        kAmplifier,
        kSpeakerAmpSide,
        kSpeakerCenterSide,
        kSpeakerSourceSide,
    }

    private final Pose2d position;
    private final DepositLocation locationIdentifier;

    public static final ArrayList<SendableChooser<NoteDepositPosition>> noteDepositList = new ArrayList<SendableChooser<NoteDepositPosition>>();
    public static SendableChooser<NoteDepositPosition> getNewNoteDepositChooser() {
        SendableChooser<NoteDepositPosition> chooser = new SendableChooser<NoteDepositPosition>();
        chooser.setDefaultOption("Speaker Center-Side", NoteDepositConstants.speakerCenterSide);
        chooser.addOption("Speaker Amp-Side", NoteDepositConstants.speakerAmpSide);
        chooser.addOption("Speaker Source-Side", NoteDepositConstants.speakerSourceSide);
        chooser.addOption("Amplifier", NoteDepositConstants.amplifier);

        noteDepositList.add(chooser);
        return chooser;
    }
    public static NoteDepositPosition getClosestDepositLocationFromPoint(Translation2d swervePosition) {
        NoteDepositPosition closestDepositLocation = null;
        double distanceOfClosestDepositLocation = 0;
        for (int i = 0; i < noteDepositList.size(); i++) {
          NoteDepositPosition currentPos = noteDepositList.get(i).getSelected();
          double distanceToCurrentPos = swervePosition.getDistance(
            currentPos.getPosition().getTranslation()
          );
          if (closestDepositLocation == null || distanceToCurrentPos < distanceOfClosestDepositLocation) {
            closestDepositLocation = currentPos;
            distanceOfClosestDepositLocation = distanceToCurrentPos;
          }
        }
        return closestDepositLocation;
      }

    public NoteDepositPosition(Pose2d positionForDeposit, DepositLocation location) {
        position = positionForDeposit;
        locationIdentifier = location;
    }

    public DepositLocation getDepositLocationEnum() {
        return locationIdentifier;
    }

    public Pose2d getPosition() {
        return position;
    }
}
