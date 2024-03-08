// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.NotePositionConstants;
import frc.robot.utils.NoteDepositPosition.DepositLocation;

/** Add your docs here. */
public class NotePosition {
    private final Translation2d kNotePosition;
    private final List<Pose2d> kAttackPositions;
    private final HashMap<DepositLocation, List<Translation2d>> kPathsToDepositories;

    public static final ArrayList<SendableChooser<NotePosition>> noteSelectionList = new ArrayList<SendableChooser<NotePosition>>();

    public static SendableChooser<NotePosition> getNewNotePositionChooser() {
        SendableChooser<NotePosition> chooser = new SendableChooser<NotePosition>();
        chooser.setDefaultOption("None", null);
        chooser.addOption("Wait 1 Sec", new WaitingNotePosition(1));
        chooser.addOption("Wait 3 Sec", new WaitingNotePosition(3));
        chooser.addOption("Wait 5 Sec", new WaitingNotePosition(5));
        chooser.addOption("Stage Wing", NotePositionConstants.StageClose);
        chooser.addOption("Center Wing", NotePositionConstants.CenterClose);
        chooser.addOption("Amp Wing", NotePositionConstants.AmpClose);
        chooser.addOption("Source First Center-Line", NotePositionConstants.SourceFirstFieldCenter);
        chooser.addOption("Source Second Center-Line", NotePositionConstants.SourceSecondFieldCenter);
        chooser.addOption("Center Center-Line", NotePositionConstants.CenterFieldCenter);
        chooser.addOption("Amp Second Center-Line", NotePositionConstants.AmpSecondFieldCenter);
        chooser.addOption("Amp First Center-Line", NotePositionConstants.AmpFirstFieldCenter);

        noteSelectionList.add(chooser);
        return chooser;
    }

    public NotePosition(
        Translation2d notePosition, 
        List<Translation2d> attackPositions, 
        HashMap<DepositLocation, List<Translation2d>> pathsToDepositories
    ) {
        kNotePosition = notePosition;
        List<Pose2d> attackPoses = new ArrayList<Pose2d>();
        kPathsToDepositories = pathsToDepositories;
        for (int i = 0; i < attackPositions.size(); i++) {
            Translation2d currentAttackPosition = attackPositions.get(i);
            
            Translation2d positionChangeFromNotePositionToAttackPosition = currentAttackPosition.minus(
                kNotePosition
            );

            Pose2d poseToSet = new Pose2d(
                currentAttackPosition, 
                new Rotation2d(
                    positionChangeFromNotePositionToAttackPosition.getX(),
                    positionChangeFromNotePositionToAttackPosition.getY()
                )
            );
            attackPoses.add(i, poseToSet);
        }
        kAttackPositions = attackPoses;
    }

    public List<Translation2d> getWaypointsNoteToDeposit(DepositLocation location) {
        List<Translation2d> waypoints = kPathsToDepositories.get(location);
        return waypoints;
    }

    public List<Translation2d> getWaypointsDepositToNote(DepositLocation location) {
        List<Translation2d> waypoints = kPathsToDepositories.get(location);
        List<Translation2d> reversedWaypoints = new ArrayList<Translation2d>();
        if (waypoints.size() <= 1) return waypoints;

        for (int i = waypoints.size()-1; i >= 0; i--) {
            reversedWaypoints.add(
                waypoints.get(i)
            );
        }
        return reversedWaypoints;
    }

    public List<Translation2d> getListOfAttackPositions() {
        List<Translation2d> returnList = List.of();

        for (int i = 0; i < kAttackPositions.size(); i++) {
            returnList.set(i, kAttackPositions.get(i).getTranslation());
        }
        return returnList;
    }

    public Pose2d getClosestAttackPosition(Translation2d startPosition) {
        if (kAttackPositions.size() == 0) return new Pose2d(kNotePosition, new Rotation2d());
        return new Pose2d(startPosition, new Rotation2d()).nearest(kAttackPositions);
    }

    public Translation2d getNotePosition() {
        return kNotePosition;
    }

    public Pose2d getNotePoseFromAttackPosition(Pose2d attackPose) {
        Translation2d addingVector = kNotePosition.minus(attackPose.getTranslation());
        double addingVectorDistance = addingVector.getNorm();
        addingVector = addingVector.times(AutoConstants.kNoteGrabDistanceOvershootMeters/addingVectorDistance);
        return new Pose2d(kNotePosition.plus(addingVector), attackPose.getRotation());
    }
}