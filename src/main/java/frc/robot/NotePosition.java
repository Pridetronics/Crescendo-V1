// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.proto.Pose2dProto;

/** Add your docs here. */
public class NotePosition {
    private final Translation2d kNotePosition;
    private final List<Pose2d> kAttackPositions;

    public NotePosition(Translation2d notePosition, List<Pose2d> attackPositions) {
        kNotePosition = notePosition;
        kAttackPositions = attackPositions;

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
