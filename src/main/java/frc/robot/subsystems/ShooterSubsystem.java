// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorCANID, MotorType.kBrushless); //This is setting our Motor
  private SparkPIDController shooterPIDController = shooterMotor.getPIDController();
  private RelativeEncoder encoder = shooterMotor.getEncoder();
  private boolean isEnabled;
  private int minimumRPM;

  private final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
  private final GenericEntry shooterEntry = teleOpTab.add("Shooter Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
//End of Class
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterPIDController.setP(ShooterConstants.kShooterPValue);
    shooterPIDController.setI(ShooterConstants.kShooterIValue);
    shooterPIDController.setD(ShooterConstants.kShooterDValue);
    shooterMotor.setInverted(true);
  } //End of Class

  public boolean isRPMOverMinimum() {
    return minimumRPM == 0 || encoder.getVelocity() >= minimumRPM;
  }
  
  //Sets the motor RPM
  public void setMotorAtRPM(int targetRPM, int newMinimumRPM) {
    shooterPIDController.setReference(targetRPM, ControlType.kVelocity);
    isEnabled = true;
    minimumRPM = newMinimumRPM;
  } //End of Class
//Stops the shooter
  public void stopMotorSpeed() {
    shooterPIDController.setReference(0, ControlType.kVelocity);
    isEnabled = false;
  } //End of Class

  public boolean isEnabled() {
    return isEnabled;
  }

  public boolean isDisabled() {
    return !isEnabled;
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    shooterEntry.setBoolean(isEnabled);
  }
} //End of Class
