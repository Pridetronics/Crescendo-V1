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
// for all of these i need to check api docs and stuff
	public void setTarget(double position) {
// use set reference method on PID controller
setreference(0,PIDController);//0 is a placeholder , i put ksmartmotion for now because it wanted a control type 
//and i was unsure if we needed position or smartmotion
	}

	public void stopClimbers() {
// use setreference with the target value to the current position
setreference(0,kPosition);//0 is a placeholder value 
	}

	public void setCurrentPosition(double position) {
//use setrefernenece with the target value to the current position
setreference(0,kPosition);
	}

	public double getPosition() {
		setPositionConversionFactor(); //done for now???
	}

	//Used for homing
	public void moveAtPercentSpeed(double speed) {
		
//use setmethod on the CANSparkMax object i created in the climber
set(climberMotor.moveAtPercentSpeed);
return 0; //this is a placeholder value for now until we get the real one
	}

	public boolean limitSwitchActivated() {
		return climberLimitSwitch.get(); //done
	}

	public void setMaxVelocity(double velocityMetersPerSecond) {
		climbController.setSmartMotionMaxVelocity(velocityMetersPerSecond, 0);
	}
}
