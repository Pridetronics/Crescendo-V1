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
  private boolean currentlyHoming;
  private boolean hasHomed;
  /** Creates a new ClimberSubsystem. */ 
  public ClimberSubsystem(SwerveSubsystem swerveSubsystem) {
    this.getRollFunction = swerveSubsystem::getGyroRoll;
    
  }
  @Override
  public void periodic() {
    if (currentlyHoming) {  
      boolean leftClimberHomed = climberLeft.updateHomingState();
      boolean rightClimberHomed = climberRight.updateHomingState();

      if (leftClimberHomed && rightClimberHomed) {
        currentlyHoming = false;
        hasHomed = true;
      }
    }
  }

  public void beginClimberHoming() {
    if (hasHomed || currentlyHoming) return;
    currentlyHoming = true;
    climberLeft.moveAtPercentSpeed(-0.05);
    climberRight.moveAtPercentSpeed(-0.05);
  }

  public void raiseClimbers() {
    if (!hasHomed) return;
    climberLeft.setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
    climberLeft.setTarget(ClimberConstants.kMaxHeightMeters);
    
    climberRight.setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
    climberRight.setTarget(ClimberConstants.kMaxHeightMeters);
  }

  public void lowerClimbers() {
    if (!hasHomed) return;
    climberLeft.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberLeft.setTarget(0);
    
    climberRight.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberRight.setTarget(0);
  }
} 


