// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final Climber climberLeft = new Climber(ClimberConstants.kClimberLeftMotorID, ClimberConstants.kClimberLeftLimitSwitchID); // this creates a new left climber object ;
  private final Climber climberRight = new Climber(ClimberConstants.kClimberRightMotorID, ClimberConstants.kClimberRightLimitSwitchID); // this creates a new right climber object; 
  private final Supplier<Double> getRollFunction;
  /** Creates a new ClimberSubsystem. */ 
  public ClimberSubsystem(SwerveSubsystem swerveSubsystem) {
    this.getRollFunction = swerveSubsystem::getGyroRoll;
    
  }
  @Override
  public void periodic() {
    if (homing) {  
      leftClimber.limitSwitchActivated(DigitalInput);
      rightClimber.limitSwitchActivated(DigitalInput);
    
    
      

      // This method will be called once per scheduler run

      //TODO IN THIS METHOD: If homing, check limit switches and handle accordingly
      //TODO IN THIS METHOD: Check roll and adjust climber speeds accordingly
    }
  }

  public void beginClimberHoming() {
    initialize();
    leftClimber.limitSwitchActivated(DigitalInput);
    rightClimber.limitSwitchActivated(DigitalInput);

  }

  public void raiseClimbers() {
    execute();
    leftClimber.raiseClimbers(DigitalInput);
    rightClimber.raiseClimbers(DigitalInput);
    
  }

    public void lowerClimbers() {
    execute();
    leftClimber.lowerClimbers(DigitalInput);
    rightClimber.lowerClimbers(DigitalInput);
  }
} 


