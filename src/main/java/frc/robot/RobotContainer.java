// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.Constants.AutoConstants.NoteDepositConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetOdometerWithCamera;
import frc.robot.commands.ShootForSeconds;
import frc.robot.commands.StopShooter;
import frc.robot.commands.FieldPositionUpdate;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.WaitForLowerIntakeSensor;
import frc.robot.commands.WindUpShooter;
import frc.robot.commands.ZeroRobotHeading;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.NoteDepositPosition;
import frc.robot.utils.NotePosition;
import frc.robot.utils.TrajectoryHelper;
import frc.robot.utils.WaitingNotePosition;

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
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final Joystick driverJoystick = new Joystick(IOConstants.kDriveJoystickID);
  private SendableChooser<Pose2d> emergencyStartingPose = new SendableChooser<Pose2d>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Optional<EstimatedRobotPose> startingRobotPose = visionSubsystem.getEstimatedPose();
    if (startingRobotPose.isPresent()) {
      swerveSubsystem.resetOdometry(startingRobotPose.get().estimatedPose.toPose2d());
    }


    //Create a shufflebaord tab for the drivers to see all teleop info
    ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
    //Create a shufflebaord tab for the drivers to see all autonomous info
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    ShuffleboardLayout notePositionLayout = autoTab.getLayout("Note Selection Order", BuiltInLayouts.kList)
    .withSize(2, 4);

    for (int i = 0; i < AutoConstants.kMaxNumberOfNotesToPickInShuffleboard; i++) {
      SendableChooser<NotePosition> createdChooser = NotePosition.getNewNotePositionChooser();
      notePositionLayout.add("Note number: " + (i+1), createdChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
      
    }

    ShuffleboardLayout notDepositLayout = autoTab.getLayout("Note Deposit Order", BuiltInLayouts.kList);

    for (int i = 0; i < AutoConstants.kMaxNumberOfNotesToPickInShuffleboard; i++) {
      SendableChooser<NoteDepositPosition> createdChooser = NoteDepositPosition.getNewNoteDepositChooser();
      notDepositLayout.add("Note number: " + (i+1), createdChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
      
    }

    emergencyStartingPose.setDefaultOption("Speaker Center-Side", NoteDepositConstants.speakerCenterSide.getPosition());
    emergencyStartingPose.addOption("Speaker Amp-Side", NoteDepositConstants.speakerAmpSide.getPosition());
    emergencyStartingPose.addOption("Speaker Source-Side", NoteDepositConstants.speakerSourceSide.getPosition());
    emergencyStartingPose.addOption("Amplifier", NoteDepositConstants.amplifier.getPosition());
    autoTab.add("Emergency Starting Pose", emergencyStartingPose);

    int simulatedYAxisMult = RobotBase.isReal() ? 1 : -1;

    //Command set to run periodicly to register joystick inputs
    //It uses suppliers/mini methods to give up to date info easily
    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickXAxis), 
        () -> driverJoystick.getRawAxis(IOConstants.kDriveJoystickYAxis) * simulatedYAxisMult, 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickTurningAxis),
        () -> !driverJoystick.getRawButton(IOConstants.kDriveFieldOrientedDriveBtnID)
      )
    );

    //Runs the command when other vision commands are not being used
    visionSubsystem.setDefaultCommand(
      new FieldPositionUpdate(
        visionSubsystem, 
        swerveSubsystem
      )
    );
    //Schedule this command so that it will update the odometer, and then allow the default command to run once it it done
    new SetOdometerWithCamera(swerveSubsystem, visionSubsystem).schedule();

    // Configure the trigger bindings
    configureBindings();
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

    new JoystickButton(driverJoystick, IOConstants.KShooterButtonID) //Setting our button to toggle the shooter
    .toggleOnTrue(new ShootForSeconds(shooterSubsystem));

    new JoystickButton(driverJoystick, IOConstants.kIntakeButtonID) //Setting our button to activate the intake while held
    .whileTrue(
      //Runs these commandws sequentially when holding button
      new SequentialCommandGroup(
        //Wait for shooter to be disabled or for velocity to be beyond a threshold before we use the intake
        new ParallelRaceGroup(
          new WaitUntilCommand(shooterSubsystem::isDisabled),
          new WaitUntilCommand(() -> shooterSubsystem.getVelocity() >= ShooterConstants.kMinRPMForIntake)
        ),
        //Run intake after previous command
        new IntakeCommand(intakeSubsystem)
      )
    );

    //Activates an Instant Command to reset field direction when button is pressed down
    new JoystickButton(driverJoystick, IOConstants.kZeroHeadingBtnID)
    .onTrue(new ZeroRobotHeading(swerveSubsystem))
    .debounce(IOConstants.kZeroHeadingDebounceTime);  

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
    for (int i = 0; i < NotePosition.noteSelectionList.size(); i++) {
      //get the note selected for the current ordered position
      NotePosition notePos = NotePosition.noteSelectionList.get(i).getSelected();

      //If the "None" option was selected then skip to the next note position order
      if (notePos == null) continue;

      //If the chosen item is an order to wait, tell the sequence to wait here for the specified period
      if (notePos instanceof WaitingNotePosition) {
        totalCommandSequence.addCommands(
          new WaitCommand(
            ((WaitingNotePosition) notePos)
            .getWaitTime()
          )
        );
        //continue to next note position
        continue;
      }

      //Get the note deposition location in the corresponding note order selected
      NoteDepositPosition currentDepositPosition = NoteDepositPosition.noteDepositList.get(i).getSelected();
      //If a previous note deposit location does not exist yet, we will set it to the closest deposit location to the robot
      if (previousOrderDepositPosition == null) {
        //If the camera is not looking at an april tag, the robot will guess its starting pose
        if (!visionSubsystem.lookingAtAprilTag()) {
          swerveSubsystem.resetOdometry(
            TrajectoryHelper.toAllianceRelativePosition(
              emergencyStartingPose.getSelected()
            )
          );
        }
        //Update the previous deposit location variable
        previousOrderDepositPosition = NoteDepositPosition.getClosestDepositLocationFromPoint(
          //Converts the robots current position to a blue alliance relative position
          TrajectoryHelper.toAllianceRelativePosition(swerveSubsystem.getPose().getTranslation())
        );
      }
      //List of waypoints in between the deposit location and the note attack position
      List<Translation2d> firstWaypoints = notePos.getWaypointsDepositToNote(previousOrderDepositPosition.getDepositLocationEnum());
      //Find the last position the robot goes to in the waypoint list
      Translation2d lastWaypointPosInList = firstWaypoints.size() > 0 ? firstWaypoints.get(firstWaypoints.size()-1) : previousOrderDepositPosition.getPosition().getTranslation();
      //Find the best attack position with the previous attack position
      Pose2d attackPosition = notePos.getClosestAttackPosition(lastWaypointPosInList);

      //Generate path from the deposit location the robot is at, to the attack position we found to be the best
      Trajectory depositToNoteAttackPos = TrajectoryHelper.createTrajectoryWithAllianceRelativePositioning(
        previousOrderDepositPosition.getPosition(), 
        firstWaypoints,
        attackPosition
      );

      //Path that goes from the attack position to the note itself
      Trajectory attackPositionToNote = TrajectoryHelper.createTrajectoryWithAllianceRelativePositioning(
        attackPosition, 
        List.of(),
        notePos.getNotePoseFromAttackPosition(attackPosition)
      );

      //Positions between the note and the next deposit location
      List<Translation2d> secondWaypoints = notePos.getWaypointsNoteToDeposit(currentDepositPosition.getDepositLocationEnum());
      //Path from the current note position to the deposit location

      Trajectory notePositionToNewDeposit = TrajectoryHelper.createTrajectoryWithAllianceRelativePositioning(
        notePos.getNotePoseFromAttackPosition(attackPosition), 
        secondWaypoints,
        currentDepositPosition.getPosition()
      );

      SequentialCommandGroup intakeDrivingSequence = new SequentialCommandGroup(
        new ParallelRaceGroup(
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
          //If the lower sensor in activated then cut off this sequence early
          new WaitForLowerIntakeSensor(intakeSubsystem)
        )
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
          //Run both the intake and driving in parallel
          new ParallelCommandGroup(
            //Runs intake
            new IntakeCommand(intakeSubsystem),
            //Drives to note and then drives to deposit location
            new SequentialCommandGroup(
              //Manuvers intake into note and collects it
              intakeDrivingSequence,
              //Does the following all at once:
              new ParallelCommandGroup(
                //Move robot to the chosen deposit area
                new SwerveControllerCommand(
                  notePositionToNewDeposit, 
                  swerveSubsystem::getPose, 
                  WheelConstants.kDriveKinematics, 
                  xController,
                  yController,
                  thetaController,
                  swerveSubsystem::setModuleStates,
                  swerveSubsystem
                ),
                //Wind up the shooter to target RPM
                new WindUpShooter(shooterSubsystem)
              ),
              new SequentialCommandGroup(
                //Intake note into shooter
                new IntakeCommand(intakeSubsystem),
                //Once intake is off, stop shooter
                new StopShooter(shooterSubsystem)
              )
            )
          )
        )
      ); //END OF COMMAND SEQUENCE FOR THIS NOTE

      previousOrderDepositPosition = currentDepositPosition;
    }

    return totalCommandSequence;
  }
}