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
	private RelativeEncoder climbeNcoder;
	private DigitalInput climbDigitalInput; 
	Climber(int motorId, int limitSwitchID) { //climber methods
		climberMotor = new CANSparkMax(motorId,CANSparkLowLevel.MotorType.kBrushless);
		climbController = climberMotor.getPIDController();
		climbeNcoder = climberMotor.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, 42);
		climbDigitalInput = new DigitalInput(limitSwitchID); 
		climbController.setSmartMotionMaxVelocity(0.2, 1);
		climbeNcoder.setPositionConversionFactor(ClimberConstants.winchCircumfrenceMeters);


	}  //end of contructor (DO NOT USE FOR NOW)

    public void setTargetPositionClimb(double targetPosition) {
		climbController.setReference(targetPosition, ControlType.kSmartMotion);
    } // end of method for target position 

	public void setAtPercentageSpeedClimb(double targetSpeed) {
		climbController.setReference(targetSpeed, ControlType.kDutyCycle);
    } // end of method for target speed 

	public void setClimbPosition(double currentPosition) {
		climbeNcoder.setPosition(currentPosition);
    } // end of method for current position


}
 // end of class bracket 




