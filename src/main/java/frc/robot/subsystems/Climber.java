// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class Climber {
	private CANSparkMax climberMotor;
	private PIDController climbController;
	private RelativeEncoder climbeNcoder;
	private DigitalInput climbDigitalInput; 
	Climber(int motorId, int limitSwitchID) {
		climberMotor = new CANSparkMax(motorId,CANSparkLowLevel.MotorType.kBrushless);
		climbController = new PIDController(motorId, motorId, motorId);
		climbeNcoder = climberMotor.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, 42);
		climbDigitalInput = new DigitalInput(limitSwitchID); 
		
	}
}
