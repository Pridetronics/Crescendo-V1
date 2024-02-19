// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.WheelConstants;
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

    //This will have commands added to it later to build the whole autonous phase
    SequentialCommandGroup totalCommandSequence = new SequentialCommandGroup();
    //To track where the robot it at when caculating paths
    NoteDepositPosition previousOrderDepositPosition = null;

    //For correcting error in driving
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //Go through all of the note dropdowns in the shuffleboard interface
    for (int i = 0; i < noteSelectionList.size(); i++) {
      //get the note selected for the current ordered position
      NotePosition notePos = noteSelectionList.get(i).getSelected();
      //Caculate the path for this ordered note position only if one was selected
      if (notePos == null) continue;

      //Get the note deposition location in the corresponding note order selected
      NoteDepositPosition currentDepositPosition = noteDepositList.get(i).getSelected();
      //If a previous note deposit location does not exist yet, we will set it to the closest deposit location to the robot
      if (previousOrderDepositPosition == null) {
        previousOrderDepositPosition = getClosestDepositLocation();
      }
      //List of waypoints in between the deposit location and the note attack position
      List<Translation2d> firstWaypoints = notePos.getWaypointsDepositToNote(previousOrderDepositPosition.getDepositLocationEnum());
      //Find the last position the robot goes to in the waypoint list
      Translation2d lastWaypointPosInList = firstWaypoints.size() > 0 ? firstWaypoints.get(firstWaypoints.size()-1) : previousOrderDepositPosition.getPosition().getTranslation();
      //Find the best attack position with the previous attack position
      Pose2d attackPosition = notePos.getClosestAttackPosition(lastWaypointPosInList);

      //Generate path from the deposit location the robot is at, to the attack position we found to be the best
      Trajectory depositToNoteAttackPos = TrajectoryGenerator.generateTrajectory(
        previousOrderDepositPosition.getPosition(), 
        firstWaypoints, 
        attackPosition, 
        Constants.kTrajectoryConfig
      );

      //Path that goes from the attack position to the note itself
      Trajectory attackPositionToNote = TrajectoryGenerator.generateTrajectory(
        attackPosition, 
        List.of(),
        notePos.getNotePoseFromAttackPosition(attackPosition), 
        Constants.kTrajectoryConfig
      );

      //Positions between the note and the next deposit location
      List<Translation2d> secondWaypoints = notePos.getWaypointsNoteToDeposit(currentDepositPosition.getDepositLocationEnum());
      //Path from the current note position to the deposit location
      Trajectory notePositionToNewDeposit = TrajectoryGenerator.generateTrajectory(
        notePos.getNotePoseFromAttackPosition(attackPosition), 
        secondWaypoints, 
        currentDepositPosition.getPosition(), 
        Constants.kTrajectoryConfig
      );

      //Adds a sequence of commands to the overall command sequence that will be returned
      totalCommandSequence.addCommands(
        new SequentialCommandGroup(
          //Move robot from the deposit it's currently at to the target note's closest attack position
          new SwerveControllerCommand(
            depositToNoteAttackPos, 
            swerveSubsystem::getPose, 
            WheelConstants.kDriveKinematics, 
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem
          ),
          //ENABLE INTAKE COMMAND HERE
          //Moves robot from the chosen attack position on the target note to the actual note itself
          new SwerveControllerCommand(
            attackPositionToNote, 
            swerveSubsystem::getPose, 
            WheelConstants.kDriveKinematics, 
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem
          ),
          //Does the following all at once:
          new ParallelCommandGroup(
            //Move robot to the chosen deposit area
            new SwerveControllerCommand(
              depositToNoteAttackPos, 
              swerveSubsystem::getPose, 
              WheelConstants.kDriveKinematics, 
              xController,
              yController,
              thetaController,
              swerveSubsystem::setModuleStates,
              swerveSubsystem
            )
            //SPIN UP SHOOTER HERE W/ WIND UP WAIT
          )
          //SHOOT NOTE HERE
        )
      );
      previousOrderDepositPosition = currentDepositPosition;
      
    }

    return totalCommandSequence;
  }
}
