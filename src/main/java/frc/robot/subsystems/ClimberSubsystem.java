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
  private final Climber climberLeft = new Climber(ClimberConstants.kClimberLeftMotorID, ClimberConstants.kClimberLeftLimitSwitchID, false); // this creates a new left climber object ;
  private final Climber climberRight = new Climber(ClimberConstants.kClimberRightMotorID, ClimberConstants.kClimberRightLimitSwitchID, true); // this creates a new right climber object; 
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
    //If the climbers have been homed OR if we are currently in the proccess of homing, we cancel the hmoming sequence
    if (hasHomed || currentlyHoming) return;
    //Stores in the subsystem a state to tell us that we a re homing
    currentlyHoming = true;
    //Sets the motors for each climber at a percent speed
    climberLeft.moveAtPercentSpeed(-0.05);
    climberRight.moveAtPercentSpeed(-0.05);
  }

  public void raiseClimbers() {
    //If the climbers have not been homed, we not raise the climbers
    if (!hasHomed) return;
    //Sets the max velocity and target height for the climbers
    climberLeft.setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
    climberLeft.setTarget(ClimberConstants.kMaxHeightMeters);
    
    climberRight.setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
    climberRight.setTarget(ClimberConstants.kMaxHeightMeters);
  }

  public void lowerClimbers() {
    //If the climbers have not been homed, we not raise the climbers
    if (!hasHomed) return;
    //Sets the max velocity and target height for the climbers
    climberLeft.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberLeft.setTarget(0);
    
    climberRight.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberRight.setTarget(0);
  }
} 


