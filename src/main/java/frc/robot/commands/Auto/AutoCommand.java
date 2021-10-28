// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Push.PushCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Trigger.TriggerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommand extends SequentialCommandGroup {
  /** Creates a new AutoCommand. */
  public AutoCommand(DriveSubsystem drive, ShooterSubsystem shooter,TriggerSubsystem trigger,PushSubsystem push) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(new ShooterCommand(shooter, -1)
    .withTimeout(5)
    .andThen(new TriggerCommand(trigger, -0.4)
    .raceWith(new PushCommand(push,-0.4),new ShooterCommand(shooter,-1))
    .withTimeout(3)
    .andThen(new BackDriveCommand(drive).withTimeout(3))));
  }
}
