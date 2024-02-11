// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.Constants.AutoConstants.NotePositionConstants;
import frc.robot.commands.SwerveAutoPaths;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroRobotHeading;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  private final Joystick driverJoystick = new Joystick(IOConstants.kDriveJoystickID);
  private final ArrayList<SendableChooser<NotePosition>> noteSelectionList = new ArrayList<SendableChooser<NotePosition>>();
  private final ArrayList<SendableChooser<Pose2d>> noteDepositList = new ArrayList<SendableChooser<Pose2d>>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    ShuffleboardLayout notePositionLayout = autoTab.getLayout("Note Selection Order", BuiltInLayouts.kList);
    for (int i = 0; i < AutoConstants.maxNumberOfNotesToPickInShuffleboard; i++) {
      SendableChooser<NotePosition> createdChooser = getNewNotePositionChooser();
      notePositionLayout.add("Note number: " + (i+1), createdChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
      noteSelectionList.add(createdChooser);
    }

    //Command set to run periodicly to register joystick inputs
    //It uses suppliers/mini methods to give up to date info easily
    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickXAxis), 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickYAxis), 
        () -> driverJoystick.getRawAxis(IOConstants.kDriveJoystickTurningAxis),
        () -> !driverJoystick.getRawButton(IOConstants.kDriveFieldOrientedDriveBtnID)
      )
    );

    // Configure the trigger bindings
    configureBindings();
  }

  public Pose2d getSwervePose() {
    return swerveSubsystem.getPose();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Activates an Instant Command to reset field direction when button is pressed down
    new JoystickButton(driverJoystick, IOConstants.kZeroHeadingBtnID)
    .onTrue(new ZeroRobotHeading(swerveSubsystem));

  }

  private SendableChooser<NotePosition> getNewNotePositionChooser() {
    SendableChooser<NotePosition> chooser = new SendableChooser<NotePosition>();
    chooser.setDefaultOption("None", null);
    chooser.addOption("Stage Close", NotePositionConstants.StageClose);
    chooser.addOption("Center Close", NotePositionConstants.CenterClose);
    chooser.addOption("Amp Close", NotePositionConstants.AmpClose);
    chooser.addOption("Source First Field Center", NotePositionConstants.SourceFirstFieldCenter);
    chooser.addOption("Source Second Field Center", NotePositionConstants.SourceSecondFieldCenter);
    chooser.addOption("Center Field Center", NotePositionConstants.CenterFieldCenter);
    chooser.addOption("Amp Second Field Center", NotePositionConstants.AmpSecondFieldCenter);
    chooser.addOption("Amp First Field Center", NotePositionConstants.AmpFirstFieldCenter);
    return chooser;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //TODO replace with auto selector
    return new Command() {

    };

  }
}
