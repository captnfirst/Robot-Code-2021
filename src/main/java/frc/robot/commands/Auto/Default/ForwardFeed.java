// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Default;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.TriggerSubsystem;

public class ForwardFeed extends CommandBase {
  /** Creates a new ForwardFeed. */
  TriggerSubsystem m_trigger;
  PushSubsystem m_push;
  double m_speed;
  public ForwardFeed(TriggerSubsystem trigger, PushSubsystem push, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_trigger=trigger;
    this.m_push=push;
    addRequirements(m_trigger,m_push);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_trigger.runTrigger(m_speed);
    m_push.runPush(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
