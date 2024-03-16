// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/** Add your docs here. */
public class ShooterWrapperCommand extends WrapperCommand {
    BooleanConsumer endFunc;

    public ShooterWrapperCommand(Command command, BooleanConsumer endRunnable) {
        super(command);
        endFunc = endRunnable;
    }

    public Command getCommand() {
        return m_command;
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
        if (endFunc != null) {
            endFunc.accept(interrupted);
        }
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
      return InterruptionBehavior.kCancelIncoming;
    }
}
