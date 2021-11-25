// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class TurretControllCommand extends CommandBase {
  /** Creates a new TurretPIDCommand. */
  private final TurretSubsystem m_turret;
  private final VisionSubsystem m_vision;
  private double error;
  private double yaw;
  private double goal;
  private String side = "";
  public TurretControllCommand(TurretSubsystem turret, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_turret = turret; 
    this.m_vision = vision;
    addRequirements(m_turret,m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Değer : " + m_vision.hasTarget());
    if(m_vision.hasTarget()){
      yaw = m_vision.getYaw();
      error = goal - yaw;
      if(error < 0){
        m_turret.runTurret(0.3);
        side = "l";
        if(error > -10){
          m_turret.runTurret(0.2);
        }
      }else if(error > 0){
        m_turret.runTurret(-0.3);
        side = "r";
        if(error < 10){
          m_turret.runTurret(-0.2);
        }
      }else{
        m_turret.runTurret(0);
      }
      if(m_turret.HeadEncoderGetValue() > -5.8){
        if(side == "r"){
          m_turret.runTurret(0);
        }
      }
      if(m_turret.HeadEncoderGetValue() < 5.8){
        if(side == "l"){
          m_turret.runTurret(0);
        }
      }
      if(error >= -2 && error <= 2){
        m_turret.runTurret(0);
      }
    }else{
      m_turret.runTurret(0);
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
    //if (m_vision.hasTarget()) return (Math.abs(m_vision.getYaw()) < 2);
    return false;
  }
}
