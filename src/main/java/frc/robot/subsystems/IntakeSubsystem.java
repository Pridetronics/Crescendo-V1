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

  public DigitalInput upperSensor = new DigitalInput(IntakeConstants.upperSensorChannelID); //Setting our upper sensor

  public DigitalInput lowerSensor = new DigitalInput(IntakeConstants.lowerSensorChannelID); //Setting our lower sensor
//** Creates the upper and lower Sensors. */

  public IntakeSubsystem() {
    intakePIDController.setP(IntakeConstants.kIntakePValue); //Calling our intake constants
  }


  public void setMotorAtRPM(double targetRPM) {
    intakePIDController.setReference(targetRPM, ControlType.kVelocity); //Setting our motor RPM
  }

  public void stopMotorSpeed() {
    intakePIDController.setReference(0, ControlType.kVelocity); //Stopping our motor
  }
  
  public void setIntakeDirection(boolean direction) { //Telling the Intake what and when our direction changes to
    intakeMotor.setInverted(direction);
  }
  
  public boolean isNoteInsideIntake() { //Checks if the note is inside the intake by using the upper sensor
     return upperSensor.get();
  }

  public boolean hasNoEnteredIntake() { //Checking if the note has entered the intake yet
    return lowerSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
