// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Default;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefaultAuto extends SequentialCommandGroup {
  Timer timer = new Timer();

  /** Creates a new DefaultAuto. */
  public DefaultAuto(
    TurretSubsystem turret, 
    ShooterSubsystem shooter,
    TriggerSubsystem trigger,
    DriveSubsystem drive,
    PushSubsystem push,
    LedSubsystem led) {
      
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      new RunCommand(()-> shooter.runShooter(-1), shooter).withTimeout(4),
      new ParallelCommandGroup(
        new RunCommand(() -> trigger.runTrigger(-0.4), trigger),
        new RunCommand(() -> push.runPush(-0.4), push),
        new RunCommand(()-> shooter.runShooter(-1), shooter)
      ){
        @Override
        public void end(boolean interrupted){
          trigger.runTrigger(0);
          push.runPush(0);
          shooter.runShooter(0);
        }
      }.withTimeout(3),
      new RunCommand(() -> drive.TankDrive(-0.5, -0.5), drive){
        @Override
        public void end(boolean interrupted){
          drive.TankDrive(0, 0);
        }
      }.withTimeout(5)
    );
  }
}
