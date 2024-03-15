// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FieldPositionUpdate extends Command {
  /** Creates a new FieldPositionUpdate. */
  VisionSubsystem m_VisionSubsystem;
  SwerveSubsystem m_SwerveSubsystem;
  private Field2d m_field = new Field2d();

  private final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
  private final ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  private final GenericEntry lookingAtAprilTag = teleOpTab.add("Looking at april Tag", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    private final GenericEntry useFieldUpdating  = teleOpTab.add("Disable Limelight", false)
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .getEntry();

  public FieldPositionUpdate(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
    m_VisionSubsystem = visionSubsystem;
    m_SwerveSubsystem = swerveSubsystem;

    SmartDashboard.putData("Field Position Visual", m_field);
    teleOpTab.add(m_field);
    autoTab.add(m_field);

    addRequirements(m_VisionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (useFieldUpdating.getBoolean(false)) return;
    //Get the caculated robot position on the field from the last camera rendering cycle
    Optional<EstimatedRobotPose> robotPose = m_VisionSubsystem.getEstimatedPose();
    //Check if a field position was caculated last cycle
    if (robotPose.isPresent()) {
      //Update the swerve drive odometry to work with this position
      m_SwerveSubsystem.addVisionMeasurement(robotPose.get().estimatedPose.toPose2d(), robotPose.get().timestampSeconds);
    }

    lookingAtAprilTag.setBoolean(robotPose.isPresent());
    
    m_field.setRobotPose(m_SwerveSubsystem.getPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //Allows the command to run when disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // //Makes it so other commands do not cancel this command
  // @Override
  // public InterruptionBehavior getInterruptionBehavior() {
  //   return InterruptionBehavior.kCancelIncoming;
  // }
}
