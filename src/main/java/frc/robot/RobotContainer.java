// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
  private final SendableChooser<Trajectory> autoCommandChooser = new SendableChooser<>();
  private final Joystick driverJoystick = new Joystick(IOConstants.kDriveJoystickID);
  private final HashMap<String, GenericEntry> noteSelectionToggles = new HashMap<String, GenericEntry>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    ShuffleboardLayout noteSelectionList = autoTab.getLayout("Notes to get");
    ShuffleboardLayout closeNoteSelectionList = noteSelectionList.getLayout("Close Notes");
    ShuffleboardLayout farNoteSelectionList = noteSelectionList.getLayout("Field Center Notes");
    ShuffleboardLayout noteDepositList = autoTab.getLayout("Deposit Notes @");
    noteSelectionToggles.put("Close Amp", 
      closeNoteSelectionList.addPersistent("Amp", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Close Center", 
      closeNoteSelectionList.addPersistent("Center", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Close Stage", 
      closeNoteSelectionList.addPersistent("Stage", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );

    noteSelectionToggles.put("Far First Amp", 
      farNoteSelectionList.addPersistent("First Amp", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Far Second Amp", 
      farNoteSelectionList.addPersistent("Second Amp", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Far Center", 
      farNoteSelectionList.addPersistent("Center", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Far Second Source", 
      farNoteSelectionList.addPersistent("Second Source", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Far First Source", 
      farNoteSelectionList.addPersistent("First Source", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );


    noteSelectionToggles.put("Speaker Amp Deposit", 
      noteDepositList.addPersistent("Speaker Amp Side", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Speaker Center Deposit", 
      noteDepositList.addPersistent("Speaker Center Side", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Speaker Source Deposit", 
      noteDepositList.addPersistent("Speaker Source Side", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    noteSelectionToggles.put("Amp Deposit", 
      noteDepositList.addPersistent("Amp", false)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry()
    );
    

    //Create your auto paths here (By which the trajectories are made in SwerveAutoPaths)
    autoCommandChooser.setDefaultOption("Do Nothing", null);
    // autoCommandChooser.addOption("test auto", SwerveAutoPaths.TestAutoPath());
    // autoCommandChooser.addOption("weird path", SwerveAutoPaths.WeirdPath());
    autoCommandChooser.addOption("Forward Right", SwerveAutoPaths.ForwardRight());

    SmartDashboard.putData("Autonomous Mode", autoCommandChooser);

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
