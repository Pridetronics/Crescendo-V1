// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.ShuffleboardRateLimiter;

public class ShooterSubsystem extends SubsystemBase {
  public final TalonFX Shooter = new TalonFX(ShooterConstants.kShooterMotorCANID); //This is setting our Motor
  public final VelocityVoltage velocity = new VelocityVoltage(0);
  public final TalonFXConfiguration configs = new TalonFXConfiguration();
  private boolean isEnabled;
  private int minimumRPM;

  private final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
  private final GenericEntry shooterEntry = teleOpTab.add("Shooter Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
//End of Class
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    configs.Slot0.kP = ShooterConstants.kShooterPValue;
    configs.Slot0.kI = ShooterConstants.kShooterIValue;
    configs.Slot0.kD = ShooterConstants.kShooterDValue;
    Shooter.getConfigurator().apply(configs, 0.050);
    velocity.Slot = 0;
    Shooter.setInverted(true);  
  }//End of Class

  public boolean isRPMOverMinimum() {
    return minimumRPM == 0 || velocity.Velocity >= minimumRPM;
  }
  
  public void setMotorAtRPM(int targetRPM, int newMinimumRPM) {
    isEnabled = true;
    Shooter.setControl(velocity.withVelocity(targetRPM));
    getVelocity();
    minimumRPM = newMinimumRPM;
  }
  
  public void stopMotorSpeed() {
    Shooter.setControl(velocity.withVelocity(0));
    isEnabled = false;
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public boolean isDisabled() {
    return !isEnabled;
  }

  public double getVelocity() {
    return velocity.Velocity;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ShuffleboardRateLimiter.queueDataForShuffleboard(shooterEntry, isEnabled);
  }
} //End of Class
