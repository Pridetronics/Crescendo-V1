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
	private boolean hasHomed;
	Climber(int motorId, int limitSwitchID) {
		climberMotor = new CANSparkMax(motorId,CANSparkLowLevel.MotorType.kBrushless);
		climbController = climberMotor.getPIDController();
		climbEncoder = climberMotor.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, 42);
		climberLimitSwitch = new DigitalInput(limitSwitchID); 
		climbController.setP(ClimberConstants.kClimberPValue);
		climbController.setI(ClimberConstants.kClimberIValue);
		climbController.setD(ClimberConstants.kClimberDValue);
		climbEncoder.setPositionConversionFactor(ClimberConstants.kWinchCircumfrenceMeters);
		setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
	}
// for all of these i need to check api docs and stuff
	public void setTarget(double position) {
		climbController.setReference(position, ControlType.kSmartMotion, 0);
	}

	public void stopClimbers() {
		// use setreference with the target value to the current position
		climbController.setReference(getPosition(), ControlType.kSmartMotion, 0);//0 is a placeholder value 
	}

	public void setCurrentPosition(double position) {
		climbEncoder.setPosition(position);
	}

	public double getPosition() {
		return climbEncoder.getPosition();
	}

	//Used for homing
	public void moveAtPercentSpeed(double speed) {
		climberMotor.set(speed);
	}

	public boolean isLimitSwitchActivated() {
		return climberLimitSwitch.get(); //done
	}

	public void setMaxVelocity(double velocityMetersPerSecond) {
		climbController.setSmartMotionMaxVelocity(velocityMetersPerSecond, 0);
	}

	public boolean updateHomingState() {
		if (!hasHomed && isLimitSwitchActivated()) {
			hasHomed = true;
			setCurrentPosition(ClimberConstants.kHomingHeightMeters);
			setTarget(ClimberConstants.kHomingHeightMeters);
		}
		return hasHomed;
	}
}