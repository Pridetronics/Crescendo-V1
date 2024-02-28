// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  Climber climberLeft;
  Climber climberRight; 

  void ClimberSubsystem() {};


   climberLeft = new Climber(ClimberConstants.climberLeftMotorID, ClimberConstants.climberLeftLimitSwitchID ); // this creates a new left climber object 
   climberRight = new Climber(ClimberConstants.climberRightMotorID, ClimberConstants.climberRightLimitSwitchID); // this creates a new right climber object

  /** Creates a new ClimberSubsystem. */ 
{}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public ClimberSubsystem(Climber climberLeft, Climber climberRight) {
    this.climberLeft = climberLeft;
    this.climberRight = climberRight;
  }
} 


