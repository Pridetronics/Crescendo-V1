// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LowerClimber;
import frc.robot.commands.RaiseClimber;
import frc.robot.commands.FieldPositionUpdate;
import frc.robot.commands.HomeClimber;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroRobotHeading;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.NoteDepositPosition;
import frc.robot.utils.NotePosition;
import frc.robot.utils.TrajectoryHelper;

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
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(swerveSubsystem);
  private final Joystick driverJoystick = new Joystick(IOConstants.kDriveJoystickID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Create a shufflebaord tab for the drivers to see all teleop info
    ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
    //Create a shufflebaord tab for the drivers to see all autonomous info
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    ShuffleboardLayout notePositionLayout = autoTab.getLayout("Note Selection Order", BuiltInLayouts.kList)
    .withSize(2, 4);


    int simulatedYAxisMult = RobotBase.isReal() ? 1 : -1;

    //Command set to run periodicly to register joystick inputs
    //It uses suppliers/mini methods to give up to date info easily
    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickXAxis), 
        () -> driverJoystick.getRawAxis(IOConstants.kDriveJoystickYAxis) * simulatedYAxisMult, 
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

    new JoystickButton(driverJoystick, IOConstants.kRaiseClimberBtnID)
    .onTrue(new RaiseClimber(climberSubsystem));

    new JoystickButton(driverJoystick, IOConstants.kLowerClimberBtnID)
    .onTrue(new LowerClimber(climberSubsystem));
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
    return new HomeClimber(climberSubsystem);
  }
}
