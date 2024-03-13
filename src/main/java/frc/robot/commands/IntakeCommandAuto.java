// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;

//import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandAuto extends Command {
  private IntakeSubsystem m_IntakeSubsystem; //Telling the system how to identify our intake subystem
  private boolean hasNoteNotBeenDetected; //Telling the system how to see our upper sensor (checking)
  private int intakeRPM;
  //end of methods
  /** Creates a new IntakeCommand. */
  public IntakeCommandAuto(IntakeSubsystem intakeSubsystem, int RPMForIntake) { //storing subsystem
    m_IntakeSubsystem = intakeSubsystem;
    intakeRPM = RPMForIntake;
    // Use addRequirements() here to declare subsystem dependencies.
  } //End of Entire Class



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.setMotorAtRPM(intakeRPM); //Sets our intake RPM by caling what we set in Robot Container
  } //End of Method

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.isNoteInsideIntake();
    if (m_IntakeSubsystem.isNoteInsideIntake() == false) { //telling the command how to use our digital inputs
      hasNoteNotBeenDetected = true;
    } //end of class
  }
//End of Class
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopMotorSpeed(); //Tells our intake what to do when it stops
    hasNoteNotBeenDetected = false;
  } //End of Class

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //For autonomous simulation
    if (RobotBase.isSimulation()) return true;

    if (m_IntakeSubsystem.isNoteInsideIntake() && hasNoteNotBeenDetected == true) { //end if the sensor(digital input) has previously been false and currently true, if not then continue running
      return true;
    }
    return false; //Shorthand for else
  }

  //Makes it so other commands do not cancel this command
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

} //End of Class
