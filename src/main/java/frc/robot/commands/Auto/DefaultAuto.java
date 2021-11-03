// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Turret.AutoAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefaultAuto extends SequentialCommandGroup {
  Timer timer = new Timer();

  /** Creates a new DefaultAuto. */
  public DefaultAuto(
    Vision vision, 
    TurretSubsystem turret, 
    ShooterSubsystem shooter,
    TriggerSubsystem trigger,
    DriveSubsystem m_drive,
    PushSubsystem push) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new AutoAlign(vision,turret),
      new RunCommand(()-> shooter.runShooter(-1), shooter).withTimeout(4),
      new ParallelCommandGroup(
        new RunCommand(() -> trigger.runTrigger(-0.4), trigger),
        new RunCommand(() -> push.runPush(-0.4), push),
        new RunCommand(()-> shooter.runShooter(-1), shooter)
      ).withTimeout(3),
      new RunCommand(() -> m_drive.TankDrive(-0.5, -0.5), m_drive){
        @Override
        public void end(boolean interrupted){
          m_drive.TankDrive(0, 0);
        }
      }.withTimeout(5)
    );
  }
}
