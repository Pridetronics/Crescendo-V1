// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.IOConstants;

/** Add your docs here. */
public class JoystickButtonIDs extends ManipulatorButtons {


    public JoystickButtonIDs() {
        kRaiseClimberBtnID = IOConstants.joystickButtonIDs.kRaiseClimberBtnID;
        kLowerClimberBtnID = IOConstants.joystickButtonIDs.kLowerClimberBtnID;

        //Sets our intake ID
        kIntakeButtonID = IOConstants.joystickButtonIDs.kIntakeButtonID;
        //Sets our shooter ID
        kShooterButtonID = IOConstants.joystickButtonIDs.kShooterButtonID;
        //Setting button ID for our amp
        kAmplifierShooterButtonID = IOConstants.joystickButtonIDs.kAmplifierShooterButtonID;
        //Setting our button ID for reversing our intake
        kReverseIntakeButtonID = IOConstants.joystickButtonIDs.kReverseIntakeButtonID;
    }
}
