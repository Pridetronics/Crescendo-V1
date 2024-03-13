// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless); //This is setting our intake motor
  private SparkPIDController intakePIDController = intakeMotor.getPIDController();

  private DigitalInput upperSensor = new DigitalInput(IntakeConstants.upperSensorChannelID);
  private DigitalInput lowerSensor = new DigitalInput(IntakeConstants.lowerSensorChannelID);

  private boolean enabledState;

  private final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
  private final GenericEntry intakeEntry = teleOpTab.add("Intake Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

  public IntakeSubsystem() {
    intakePIDController.setP(IntakeConstants.kIntakePValue); //V
    intakePIDController.setI(IntakeConstants.kIntakeIValue); //V
    intakePIDController.setD(IntakeConstants.kIntakeDValue); //Calling our intake constants (values we stated as well)
    intakeMotor.setIdleMode(IdleMode.kBrake); //Sets our idle mode on our controller to brake mode
  }//End of Method

  public void setMotorAtRPM(double targetRPM) {
    intakePIDController.setReference(targetRPM, ControlType.kVelocity); //Setting our motor RPM
    enabledState = true;
  } //End of Method

  public void stopMotorSpeed() {
    intakeMotor.set(0); //Stopping our motor
    enabledState = false;
  } //End of Method
  
  public void setIntakeDirection(boolean direction) { //Telling the Intake what and when our direction changes to
    intakeMotor.setInverted(direction);
  } //End of Method
  
  public boolean isNoteInsideIntake() { //Checks if the note is inside the intake (Using Upper Sensor))
     return !upperSensor.get();
  } //End of Class

  public boolean hasNoEnteredIntake() { //Checking if the note has entered the intake yet (Using Lower Sensor)
    return !lowerSensor.get();
  } //End of Method

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    intakeEntry.setBoolean(enabledState);
  }
}
//End of Method