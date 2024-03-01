// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;

//import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private IntakeSubsystem m_IntakeSubsystem; //Telling the system how to identify our intake subystem
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.setMotorAtRPM(Constants.IntakeConstants.intakeRPM); //Sets our intake RPM by caling what we set in Robot Container
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopMotorSpeed(); //Tells our intake to stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSubsystem.isNoteInsideIntake(); //Checks if the note is inside the intake
  }
}
