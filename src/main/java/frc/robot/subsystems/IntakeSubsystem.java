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
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless); //This is setting our intake motor
  private SparkPIDController intakePIDController = intakeMotor.getPIDController();
  /** Creates a new IntakeSubsystem. ^ */ 
//End of Class
  public DigitalInput upperSensor = new DigitalInput(IntakeConstants.upperSensorChannelID);

  public DigitalInput lowerSensor = new DigitalInput(IntakeConstants.lowerSensorChannelID);
//** Creates the upper and lower Sensors. ^ */
//End of Method
  public IntakeSubsystem() {
    intakePIDController.setP(IntakeConstants.kIntakePValue);
    intakePIDController.setI(IntakeConstants.kIntakeIValue);
    intakePIDController.setD(IntakeConstants.kIntakeDValue); //Calling our intake constants
  }
//End of Method

  public void setMotorAtRPM(double targetRPM) {
    intakePIDController.setReference(targetRPM, ControlType.kVelocity); //Setting our motor RPM
  } //End of Method

  public void stopMotorSpeed() {
    intakePIDController.setReference(0, ControlType.kVelocity); //Stopping our motor
  } //End of Class
  
  public void setIntakeDirection(boolean direction) { //Telling the Intake what and when our direction changes to
    intakeMotor.setInverted(direction);
  } //End of Class
  
  public boolean isNoteInsideIntake() { //Checks if the note is inside the intake (Using Upper Sensor))
     return upperSensor.get();
  } //End of Class

  public boolean hasNoEnteredIntake() { //Checking if the note has entered the intake yet (Using Lower Sensor)
    return lowerSensor.get();
  } //End of Class

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
//End of Method