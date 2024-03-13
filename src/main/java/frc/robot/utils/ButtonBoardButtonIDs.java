// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.IOConstants;

/** Add your docs here. */
public class ButtonBoardButtonIDs extends ManipulatorButtons {


    public ButtonBoardButtonIDs() {
        kRaiseClimberBtnID = IOConstants.buttonBoardButtonIDs.kRaiseClimberBtnID;
        kLowerClimberBtnID = IOConstants.buttonBoardButtonIDs.kLowerClimberBtnID;

        //Sets our intake ID
        kIntakeButtonID = IOConstants.buttonBoardButtonIDs.kIntakeButtonID;
        //Sets our shooter ID
        kShooterButtonID = IOConstants.buttonBoardButtonIDs.kShooterButtonID;
        //Setting button ID for our amp
        kAmplifierShooterButtonID = IOConstants.buttonBoardButtonIDs.kAmplifierShooterButtonID;
        //Setting our button ID for reversing our intake
        kReverseIntakeButtonID = IOConstants.buttonBoardButtonIDs.kReverseIntakeButtonID;
    }
}
