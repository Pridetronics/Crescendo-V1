// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;

public class DisplayConditionCommand extends Command {
  private final GenericEntry m_entry;
  private final Supplier<Boolean> m_condition;
  /** 
   * Creates a command that will set the given boolean entry based on the condition.  The command will run until the condition is true or until interrupted.
   * 
   * <p>The entry is displayed as false until the condition has as any point become true.
   * 
   * @param entry The boolean network entry to update.
   * @param condition The condition to check every execution phase.
   * */
  public DisplayConditionCommand(GenericEntry entry, Supplier<Boolean> condition) {
    m_entry = entry;
    m_condition = condition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Starts off by setting the entry to false
    m_entry.setBoolean(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //If the end method was called without interuption, then the condition has to be true
    if (!interrupted) {
      m_entry.setBoolean(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Tells the scheduler to finish the command when the condition is true
    return m_condition.get();
  }
}
