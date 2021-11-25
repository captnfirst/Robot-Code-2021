// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Default;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Push.PushCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Trigger.TriggerCommand;
import frc.robot.commands.Turret.TurretControllCommand;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoManuallyCommand extends SequentialCommandGroup {
  /** Creates a new AutoManuallyCommand. */
  public AutoManuallyCommand(ShooterSubsystem shooter,TriggerSubsystem trigger,PushSubsystem push, TurretSubsystem turret, VisionSubsystem vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(new TurretControllCommand(turret, vision),new ShooterCommand(shooter, -1).withTimeout(4).andThen(new TriggerCommand(trigger, -0.4).raceWith(new PushCommand(push,-0.4),new ShooterCommand(shooter,-1))));
  }
}
