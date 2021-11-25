// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmIntake.IntakeGroupCommand;
import frc.robot.commands.Auto.Drive.BackAngleCommand;
import frc.robot.commands.Auto.Drive.BackDriveCommand;
import frc.robot.commands.Auto.Drive.ForwardAngleCommand;
import frc.robot.commands.Auto.Drive.ForwardDriveCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Turret.TurretControllCommand;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  public Autonomous(TurretSubsystem turret, VisionSubsystem vision, ShooterSubsystem shooter, TriggerSubsystem trigger, PushSubsystem push, DriveSubsystem m_drivetrain, ArmIntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(new TurretControllCommand(turret, vision).alongWith(new ShooterCommand(shooter, 1).withTimeout(4))
    ,new ParallelCommandGroup(
      new RunCommand(() -> trigger.runTrigger(-0.4), trigger),
      new RunCommand(() -> push.runPush(-0.4), push),
      new RunCommand(()-> shooter.runShooter(1), shooter)
    ){
      @Override
      public void end(boolean interrupted){
        trigger.runTrigger(0);
        push.runPush(0);
        shooter.runShooter(0);
      }
    }.withTimeout(3),
    new BackAngleCommand(m_drivetrain), new IntakeGroupCommand(intake, push).alongWith(new BackDriveCommand(m_drivetrain)).withTimeout(2),
    new ForwardDriveCommand(m_drivetrain).withTimeout(2) ,new ForwardAngleCommand(m_drivetrain),
    new TurretControllCommand(turret, vision),
    new ParallelCommandGroup(
      new RunCommand(() -> trigger.runTrigger(-0.4), trigger),
      new RunCommand(() -> push.runPush(-0.4), push),
      new RunCommand(()-> shooter.runShooter(1), shooter)
    ){
      @Override
      public void end(boolean interrupted){
        trigger.runTrigger(0);
        push.runPush(0);
        shooter.runShooter(0);
      }
    }.withTimeout(3));
  }
}
