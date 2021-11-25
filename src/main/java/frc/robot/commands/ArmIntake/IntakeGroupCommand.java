// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmIntake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Push.PushCommand;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.PushSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeGroupCommand extends SequentialCommandGroup {
  /** Creates a new IntakeGroupCommand. */
  public IntakeGroupCommand(ArmIntakeSubsystem intake, PushSubsystem push) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ArmIntakeCommand(intake, 0.8).alongWith(new PushCommand(push, -0.8)));
  }
}
