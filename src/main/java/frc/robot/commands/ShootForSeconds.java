// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;

public class ShootForSeconds extends Command {
  private ShooterSubsystem m_ShooterSubsystem; //Telling the system how to identify our shooter subsystem
  private Timer ShooterTimer = new Timer(); //Setting a timer for our shooter
  private double timeToShootFor;
  private int shootRPM;
  private int shooterMinimumRPM;
  /** Creates a new ShootShooterCmd. */
  public ShootForSeconds(ShooterSubsystem shooterSubsystem, double timeEnabledFor, int shooterRPM, int minimumRPM) {
    m_ShooterSubsystem = shooterSubsystem;
    timeToShootFor = timeEnabledFor;
    shootRPM = shooterRPM;
    shooterMinimumRPM = minimumRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
  } //End of Class

  public double getTargetRPM() {
    return shootRPM;
  }

  public int getMinimumRPM() {
    return shooterMinimumRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setMotorAtRPM(shootRPM, getMinimumRPM()); //Setting our shooter rpm
    ShooterTimer.start();
    ShooterTimer.reset();  //This starts our timer for the shooter running, then resets it (Line above this is included)
  } //End of Class

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
 //End of Class
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stopMotorSpeed();
  }
 //End of Class
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterTimer.hasElapsed(timeToShootFor); //Sets when our timer will stop
  }

  //Makes it so other commands do not cancel this command
  // @Override
  // public InterruptionBehavior getInterruptionBehavior() {
  //   return InterruptionBehavior.kCancelIncoming;
  // }
} //End of Class
