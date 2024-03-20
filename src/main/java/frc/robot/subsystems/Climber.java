// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimberConstants;

/** Add your docs here. */
public class Climber {
	private CANSparkMax climberMotor;
	private SparkPIDController climbController;
	private RelativeEncoder climbEncoder;
	private DigitalInput climberLimitSwitch;
	private boolean hasHomed;
	Climber(int motorId, int limitSwitchID, boolean reversed) {
		climberMotor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushless);
		climbController = climberMotor.getPIDController();
		climbEncoder = climberMotor.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, 42);
		climberLimitSwitch = new DigitalInput(limitSwitchID); 
		climbController.setP(ClimberConstants.kClimberPValue);
		climbController.setI(ClimberConstants.kClimberIValue);
		climbController.setD(ClimberConstants.kClimberDValue);
		climbEncoder.setPositionConversionFactor(ClimberConstants.kWinchCircumfrenceMeters*ClimberConstants.kClimberGearRatio);
		setMaxVelocity(ClimberConstants.kMaxVelocityWhenRaisingMetersPerSecond);
		climberMotor.setInverted(reversed);
	}

	public static double convertMetersToWinchMeters(double meters) {
		return Units.inchesToMeters(-Math.sqrt((Units.metersToInches(meters)-13.7966)/-0.0799851)+13.1);
	}

	//Sets the target point for the PID controller
	public void setTarget(double position) {
		climbController.setReference(position < Units.inchesToMeters(.3) ? 0 : convertMetersToWinchMeters(position), ControlType.kSmartMotion, 0);
	}
	//Tells the climbres to stop where they currently are
	public void stopClimber() {
		// use setreference with the target value to the current position
		climbController.setReference(getPosition(), ControlType.kSmartMotion, 0);//0 is a placeholder value 
	}
	//Changes the encoder position of where the climber is currently at to the given position
	public void setCurrentPosition(double position) {
		climbEncoder.setPosition(position);
	}
	//Returns the current position of the encoder
	public double getPosition() {
		return climbEncoder.getPosition();
	}

	//Moves the motor at a percent speed (without a PID controller)
	public void moveAtPercentSpeed(double speed) {
		climberMotor.set(speed);
	}

	//Returns the state of the limit switch
	public boolean isLimitSwitchActivated() {
		return !climberLimitSwitch.get(); //done
	}

	//Sets the maximum velocity
	public void setMaxVelocity(double velocityMetersPerSecond) {
		//Sets the velocity and accleration by converting the units from meters to RPM
		climbController.setSmartMotionMaxVelocity(velocityMetersPerSecond/climbEncoder.getPositionConversionFactor()*60.0, 0);
		climbController.setSmartMotionMaxAccel(velocityMetersPerSecond/climbEncoder.getPositionConversionFactor()*60.0, 0);
	}

	//Used by subsystem to automaticly handle the homing sequence for ths climber.  Returns if the homing phase has finished
	public boolean updateHomingState() {
		//Continue only if the homing phase has not finished and the limit switch has activated
		if (!hasHomed && isLimitSwitchActivated()) {
			//Store a boolean that we have finihsed homing
			hasHomed = true;
			//Sets the current position to the height the climber is at when triggered by the sensor
			setCurrentPosition(ClimberConstants.kHomingHeightMeters);
			//Sets the target position to aim for to be the the lowest height possible
			setTarget(0);
		}
		return hasHomed;
	}
}
