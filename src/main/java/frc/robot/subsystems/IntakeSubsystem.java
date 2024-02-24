// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless); //This is setting our intake motor
  private SparkPIDController intakePIDController = intakeMotor.getPIDController();
  /** Creates a new IntakeSubsystem. */

  public DigitalInput upperSensor = new DigitalInput(IntakeConstants.upperSensorChannelID);

  public DigitalInput lowerSensor = new DigitalInput(IntakeConstants.lowerSensorChannelID);
//** Creates the upper and lower Sensors. */

  public IntakeSubsystem() {
    intakePIDController.setP(IntakeConstants.kIntakePValue);
  }


  public void setMotorAtRPM(double targetRPM) {
    intakePIDController.setReference(targetRPM, ControlType.kVelocity);
  }

  public void stopMotorSpeed() {
    intakePIDController.setReference(0, ControlType.kVelocity);
  }
  
  public void setIntakeDirection(boolean direction) {
    intakeMotor.setInverted(direction);
  }
  
  public boolean isNoteInsideIntake() {
     return upperSensor.get();
  }

  public boolean hasNoEnteredIntake() {
    return lowerSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
