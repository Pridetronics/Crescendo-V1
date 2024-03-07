// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimberConstants;

/** Add your docs here. */
public class Climber {
	private CANSparkMax climberMotor;
	private SparkPIDController climbController;
	private RelativeEncoder climbEncoder;
	private DigitalInput climberLimitSwitch; 
	Climber(int motorId, int limitSwitchID) {
		climberMotor = new CANSparkMax(motorId,CANSparkLowLevel.MotorType.kBrushless);
		climbController = climberMotor.getPIDController();
		climbEncoder = climberMotor.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, 42);
		climberLimitSwitch = new DigitalInput(limitSwitchID); 

		climbController.setP(ClimberConstants.kClimberPValue);
		climbController.setI(ClimberConstants.kClimberIValue);
		climbController.setD(ClimberConstants.kClimberDValue);
	}

	public void setTarget(double position) {

	}

	public void stopClimbers() {

	}

	public void setCurrentPosition(double position) {

	}

	public double getPosition() {
		return 0; //TODO: replace with actual numbers later (Currently a placeholder value)
	}

	//Used for honing
	public void moveAtPercentSpeed(double speed) {

	}

	public boolean limitSwitchActivated() {
		return false; //TODO: replace with actual value later (Currently a placeholder value)
	}

	public void setMaxVelocity(double velocityMetersPerSecond) {
		climbController.setSmartMotionMaxVelocity(velocityMetersPerSecond, 0);
	}
}
