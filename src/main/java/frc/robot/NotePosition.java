// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.NoteDepositPosition.DepositLocation;

/** Add your docs here. */
public class NotePosition {
    private final Translation2d kNotePosition;
    private final List<Pose2d> kAttackPositions;
    private final HashMap<DepositLocation, List<Translation2d>> kPathsToDepositories;

    public NotePosition(
        Translation2d notePosition, 
        List<Translation2d> attackPositions, 
        HashMap<DepositLocation, List<Translation2d>> pathsToDepositories
    ) {
        kNotePosition = notePosition;
        List<Pose2d> attackPoses = List.of();
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
            attackPoses.set(i, poseToSet);
        }
        kAttackPositions = attackPoses;
        
    }

    public List<Translation2d> getListOfAttackPositions() {
        List<Translation2d> returnList = List.of();

        for (int i = 0; i < kAttackPositions.size(); i++) {
            returnList.set(i, kAttackPositions.get(i).getTranslation());
        }
        return returnList;
    }

    public Pose2d getClosestAttackPosition(Translation2d startPosition) {
        return new Pose2d(startPosition, new Rotation2d()).nearest(kAttackPositions);
    }
}
