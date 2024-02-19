// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class NoteDepositPosition {
    enum DepositLocation {
        kAmplifier,
        kSpeakerAmpSide,
        kSpeakerCenterSide,
        kSpeakerSourceSide,
    }

    private final Pose2d position;
    private final DepositLocation locationIdentifier;

    NoteDepositPosition(Pose2d positionForDeposit, DepositLocation location) {
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
