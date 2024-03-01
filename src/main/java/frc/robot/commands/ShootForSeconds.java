// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;

public class ShootForSeconds extends Command {
  private ShooterSubsystem m_ShooterSubsystem;
  private Timer ShooterTimer = new Timer(); //Setting a timer for our shooter
  /** Creates a new ShootShooterCmd. */
  public ShootForSeconds(ShooterSubsystem shooterSubsystem) {
    m_ShooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
  } //Command not ready First step, tell motor to start moving on initialize, second step, tell motor to stop moving in the end, third step, tell command to stop in finished when a certain amount of time has passed

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setMotorAtRPM(Constants.ShooterConstants.shooterRPM);
    ShooterTimer.start();
    ShooterTimer.reset();  //This starts our timer for the shooter running, then resets it (Line above this is included)
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
    return ShooterTimer.hasElapsed(Constants.ShooterConstants.TimeToShootSeconds); //Sets when our timer will stop
  }
}
