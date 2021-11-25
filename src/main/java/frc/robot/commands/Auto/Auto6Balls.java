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
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.ShooterSubsystem; 
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto6Balls extends SequentialCommandGroup {
  /** Creates a new Auto8Balls. */
  
  public Auto6Balls(DriveSubsystem m_drivetrain, VisionSubsystem m_vision, ArmIntakeSubsystem m_intake, TurretSubsystem m_turret, ShooterSubsystem m_shooter,TriggerSubsystem m_trigger,PushSubsystem m_push,LedSubsystem m_led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new TurretControllCommand(m_turret,m_vision).withTimeout(1).alongWith(new ShooterCommand(m_shooter, -1)).withTimeout(4),
      // TurretPIDCommand yerine new PID yazdım onu koydum çalışmazsa değiş
      new ParallelCommandGroup(
      new RunCommand(() -> m_trigger.runTrigger(-0.4), m_trigger),
      new RunCommand(() -> m_push.runPush(-0.4), m_push),
      new RunCommand(()-> m_shooter.runShooter(-1), m_shooter)
    ){
      @Override
      public void end(boolean interrupted){
        m_trigger.runTrigger(0);
        m_push.runPush(0);
        m_shooter.runShooter(0);
      }
    }.withTimeout(2),
      new BackAngleCommand(m_drivetrain).withTimeout(1), new IntakeGroupCommand(m_intake, m_push).alongWith(new BackDriveCommand(m_drivetrain)).withTimeout(2),
      new ForwardDriveCommand(m_drivetrain).withTimeout(2) ,new ForwardAngleCommand(m_drivetrain).withTimeout(1),
      new ParallelCommandGroup(new TurretControllCommand(m_turret,m_vision),
    //bunu ekledim yeni olarak eğer çalışmazsa silersin 
      new RunCommand(() -> m_trigger.runTrigger(-0.4), m_trigger),
      new RunCommand(() -> m_push.runPush(-0.4), m_push),
      new RunCommand(()-> m_shooter.runShooter(-1), m_shooter)
    ){
      @Override
      public void end(boolean interrupted){
        m_trigger.runTrigger(0);
        m_push.runPush(0);
        m_shooter.runShooter(0);
      }
    }.withTimeout(3)
    
    );
  }
}
