// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.NoteDepositPosition.DepositLocation;

/** Add your docs here. */
public class WaitingNotePosition extends NotePosition {
    private double timeToWait;

    public WaitingNotePosition(double waitTime) {
        super(
            new Translation2d(), 
            List.of(), 
            new HashMap<DepositLocation, List<Translation2d>>()
        );
        timeToWait = waitTime;
    }

    public double getWaitTime() {
        return timeToWait;
    }
}
