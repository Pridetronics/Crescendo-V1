// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SetOdometerWithCamera extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  boolean hasReset = false;

  /** Creates a new SetOdometerWithCamera. */
  public SetOdometerWithCamera(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    m_SwerveSubsystem = swerveSubsystem;
    m_VisionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> startingRobotPose = m_VisionSubsystem.getEstimatedPose();
    if (startingRobotPose.isPresent()) {
      hasReset = true;
      m_SwerveSubsystem.resetOdometry(startingRobotPose.get().estimatedPose.toPose2d());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasReset || DriverStation.isEnabled();
  }
}
