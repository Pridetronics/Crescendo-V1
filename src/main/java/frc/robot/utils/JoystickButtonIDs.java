// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants.IOConstants;

/** Add your docs here. */
public class JoystickButtonIDs extends ManipulatorButtons {
    public final int kRaiseClimberBtnID = IOConstants.joystickButtonIDs.kRaiseClimberBtnID;
    public final int kLowerClimberBtnID = IOConstants.joystickButtonIDs.kLowerClimberBtnID;

    //Sets our intake ID
    public final int kIntakeButtonID = IOConstants.joystickButtonIDs.kIntakeButtonID;
    //Sets our shooter ID
    public final int kShooterButtonID = IOConstants.joystickButtonIDs.kShooterButtonID;
    //Setting button ID for our amp
    public final int kAmplifierShooterButtonID = IOConstants.joystickButtonIDs.kAmplifierShooterButtonID;
    //Setting our button ID for reversing our intake
    public final int kReverseIntakeButtonID = IOConstants.joystickButtonIDs.kReverseIntakeButtonID;
}
