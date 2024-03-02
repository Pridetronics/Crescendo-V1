// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorCANID, MotorType.kBrushless); //This is setting our Motor
  SparkPIDController shooterPIDController = shooterMotor.getPIDController();
//End of Class
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterPIDController.setP(ShooterConstants.kShooterPValue);
    shooterPIDController.setI(ShooterConstants.kShooterIValue);
    shooterPIDController.setD(ShooterConstants.kShooterDValue);
  } //End of Class
  
  //Sets the motor RPM
  public void setMotorAtRPM(double targetRPM) {
    shooterPIDController.setReference(targetRPM, ControlType.kVelocity);
  } //End of Class
//Stops the shooter
  public void stopMotorSpeed() {
    shooterPIDController.setReference(0, ControlType.kVelocity);
  } //End of Class

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} //End of Class
