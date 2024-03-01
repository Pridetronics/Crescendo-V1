// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class HomeClimber extends Command {
  private ClimberSubsystem m_ClimberSubsystem;
   // Creates a new ClimberCommand
   public HomeClimber(ClimberSubsystem climberSubsystem) {
m_ClimberSubsystem = climberSubsystem;

addRequirements(m_ClimberSubsystem);
   }
  /** Creates a new HomeClimber. */
  public HomeClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
