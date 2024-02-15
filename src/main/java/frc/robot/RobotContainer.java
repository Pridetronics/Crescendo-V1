// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoConstants.NoteDepositConstants;
import frc.robot.Constants.AutoConstants.NotePositionConstants;
import frc.robot.NoteDepositPosition.DepositLocation;
import frc.robot.commands.FieldPositionUpdate;
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
  private final ArrayList<SendableChooser<NoteDepositPosition>> noteDepositList = new ArrayList<SendableChooser<NoteDepositPosition>>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Create a shufflebaord tab for the drivers to see all autonomous info
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    //Get a list that will hold all list items
    ShuffleboardLayout notePositionLayout = autoTab.getLayout("Note Selection Order", BuiltInLayouts.kList);
    //Add a dropdown to choose which notes are going to be collected in what order
    for (int i = 0; i < AutoConstants.maxNumberOfNotesToPickInShuffleboard; i++) {
      SendableChooser<NotePosition> createdChooser = getNewNotePositionChooser();
      notePositionLayout.add("Note number: " + (i+1), createdChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
      noteSelectionList.add(createdChooser);
    }

    ShuffleboardLayout notDepositLayout = autoTab.getLayout("Note Deposit Order", BuiltInLayouts.kList);

    for (int i = 0; i < AutoConstants.maxNumberOfNotesToPickInShuffleboard; i++) {
      SendableChooser<NoteDepositPosition> createdChooser = getNewNoteDepositChooser();
      notDepositLayout.add("Note number: " + (i+1), createdChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
      noteDepositList.add(createdChooser);
    }

    //Command set to run periodicly to register joystick inputs
    //It uses suppliers/mini methods to give up to date info easily
    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickXAxis), 
        () -> driverJoystick.getRawAxis(IOConstants.kDriveJoystickYAxis), 
        () -> driverJoystick.getRawAxis(IOConstants.kDriveJoystickTurningAxis),
        () -> !driverJoystick.getRawButton(IOConstants.kDriveFieldOrientedDriveBtnID)
      )
    );

    visionSubsystem.setDefaultCommand(
      new FieldPositionUpdate(
        visionSubsystem, 
        swerveSubsystem
      )
    );

    // Configure the trigger bindings
    configureBindings();
  }

  public Pose2d getSwervePose() {
    return new Pose2d(swerveSubsystem.getPose().getTranslation(), Rotation2d.fromDegrees(swerveSubsystem.simulatedRobotAngle));
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
    .onTrue(new ZeroRobotHeading(swerveSubsystem))
    .debounce(IOConstants.kZeroHeadingDebounceTime);

  }

  private SendableChooser<NotePosition> getNewNotePositionChooser() {
    SendableChooser<NotePosition> chooser = new SendableChooser<NotePosition>();
    chooser.setDefaultOption("None", null);
    chooser.addOption("Stage Wing", NotePositionConstants.StageClose);
    chooser.addOption("Center Wing", NotePositionConstants.CenterClose);
    chooser.addOption("Amp Wing", NotePositionConstants.AmpClose);
    chooser.addOption("Source First Center-Line", NotePositionConstants.SourceFirstFieldCenter);
    chooser.addOption("Source Second Center-Line", NotePositionConstants.SourceSecondFieldCenter);
    chooser.addOption("Center Center-Line", NotePositionConstants.CenterFieldCenter);
    chooser.addOption("Amp Second Center-Line", NotePositionConstants.AmpSecondFieldCenter);
    chooser.addOption("Amp First Center-Line", NotePositionConstants.AmpFirstFieldCenter);
    return chooser;
  }

  private SendableChooser<NoteDepositPosition> getNewNoteDepositChooser() {
    SendableChooser<NoteDepositPosition> chooser = new SendableChooser<NoteDepositPosition>();
    chooser.setDefaultOption("Stage Center-Side", NoteDepositConstants.speakerCenterSide);
    chooser.addOption("Stage Amp-Side", NoteDepositConstants.speakerAmpSide);
    chooser.addOption("Stage Source-Side", NoteDepositConstants.speakerSourceSide);
    chooser.addOption("Amplifier", NoteDepositConstants.amplifier);
    return chooser;
  }

  private NoteDepositPosition getClosestDepositLocation() {
    //TODO make this work
    NoteDepositPosition closestDepositLocation = null;
    double distanceOfClosestDepositLocation = 0;
    for (int i = 0; i < noteDepositList.size(); i++) {
      NoteDepositPosition currentPos = noteDepositList.get(i).getSelected();
      double distanceToCurrentPos = swerveSubsystem.getPose().getTranslation().getDistance(
        currentPos.getPosition().getTranslation()
      );
      if (closestDepositLocation == null || distanceToCurrentPos < distanceOfClosestDepositLocation) {
        closestDepositLocation = currentPos;
        distanceOfClosestDepositLocation = distanceToCurrentPos;
      }
    }
    return closestDepositLocation;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SequentialCommandGroup totalCommandSequence = new SequentialCommandGroup();
    NoteDepositPosition lastDepositLocation = null;
    for (int i = 0; i < noteSelectionList.size(); i++) {
      NotePosition notePos = noteSelectionList.get(i).getSelected();
      if (notePos != null) {
        NoteDepositPosition depositLocation = noteDepositList.get(i).getSelected();
        if (lastDepositLocation == null) {
          lastDepositLocation = getClosestDepositLocation();
        }
      }
    }

    return totalCommandSequence;
  }
}
