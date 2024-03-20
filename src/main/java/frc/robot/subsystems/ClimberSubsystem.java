// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private static enum climberState {
    kRaising,
    kLowering,
    kStopped,
    kHoming,
    kNonFunctional
  }

  private final Climber climberLeft = new Climber(ClimberConstants.kClimberLeftMotorID, ClimberConstants.kClimberLeftLimitSwitchID, false); // this creates a new left climber object ;
  private final Climber climberRight = new Climber(ClimberConstants.kClimberRightMotorID, ClimberConstants.kClimberRightLimitSwitchID, true); // this creates a new right climber object; 

  private final Supplier<Double> getRollFunction;
  private climberState state = climberState.kNonFunctional;
  
  /** Creates a new ClimberSubsystem. */ 
  public ClimberSubsystem(SwerveSubsystem swerveSubsystem) {
    this.getRollFunction = swerveSubsystem::getGyroRoll;
    
  }
  @Override
  public void periodic() {

    if (state == climberState.kHoming) {  
      boolean leftClimberHomed = climberLeft.updateHomingState();
      boolean rightClimberHomed = climberRight.updateHomingState();

      if (leftClimberHomed && rightClimberHomed) {
        state = climberState.kStopped;
      }
    } else if (state == climberState.kLowering) {

      if (climberLeft.isLimitSwitchActivated() || climberRight.isLimitSwitchActivated()) {
        stopClimbers();
        return;
      }

      double degreesOfRoll = getRollFunction.get();

      climberLeft.setMaxVelocity(
        ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond*( 1 + degreesOfRoll*-ClimberConstants.kProportionalVelocityChangePerDegreeOfRoll)
      );
      climberRight.setMaxVelocity(
        ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond*( 1 + degreesOfRoll*ClimberConstants.kProportionalVelocityChangePerDegreeOfRoll)
      );
    } else if (state == climberState.kStopped) {
      //TODO: Balance robot when stopped for when another robot hops on the chain
    }
  }

  public void beginClimberHoming() {
    //If the climbers have been homed OR if we are currently in the proccess of homing, we cancel the hmoming sequence
    if (state != climberState.kNonFunctional) return;
    //Stores in the subsystem a state to tell us that we a re homing
    state = climberState.kHoming;
    //Sets the motors for each climber at a percent speed
    climberLeft.moveAtPercentSpeed(-0.05d);
    climberRight.moveAtPercentSpeed(-0.05d);
  }

  public void raiseClimbers() {
    //If the climbers have not been homed, we not raise the climbers
    if (state == climberState.kHoming || state == climberState.kNonFunctional) return;
    state = climberState.kRaising;
    //Sets the max velocity and target height for the climbers
    climberLeft.setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
    climberLeft.setTarget(ClimberConstants.kMaxHeightMeters);
    
    climberRight.setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
    climberRight.setTarget(ClimberConstants.kMaxHeightMeters);
  }

  public void lowerClimbers() {
    //If the climbers have not been homed, we not raise the climbers
    if (state == climberState.kHoming || state == climberState.kNonFunctional) return;
    state = climberState.kLowering;
    //Sets the max velocity and target height for the climbers
    climberLeft.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberLeft.setTarget(Units.inchesToMeters(0.3));
    
    climberRight.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberRight.setTarget(Units.inchesToMeters(0.3));
  }

  public void stopClimbers() {
    //If the climbers have not been homed, we not stop the climbers
    if (state == climberState.kHoming || state == climberState.kNonFunctional) return;
    state = climberState.kStopped;
    //Sets the max velocity and target height for the climbers
    climberLeft.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberLeft.stopClimber();
    
    climberRight.setMaxVelocity(ClimberConstants.kMaxVelocityWhenLoweringMetersPerSecond);
    climberRight.stopClimber();
  }
} 


