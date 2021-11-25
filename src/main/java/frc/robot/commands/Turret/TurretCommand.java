// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
  /** Creates a new TurretCommand. */
  double m_speed;
  TurretSubsystem m_turret;
  boolean IsFinished = false;
  public TurretCommand(TurretSubsystem turret,double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_speed= speed;
    this.m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_speed<0){
      if(m_turret.HeadEncoderGetValue() > -4.1){
        m_turret.runTurret(m_speed);
      }else{
        IsFinished = true;
      }
    }else if(m_speed>0){
      if(m_turret.HeadEncoderGetValue() < 4.1){
        m_turret.runTurret(m_speed);
      }else{
        IsFinished = true;
      }
    }else{
      IsFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.runTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
