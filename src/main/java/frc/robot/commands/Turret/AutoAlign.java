// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  private final TurretSubsystem m_turret;
  private final LedSubsystem m_led;
  private double error;
  private double yaw;
  private double goal;
  private String side = "";
  int accuracy;
  double x;

  public AutoAlign(TurretSubsystem turret, LedSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_turret = turret; 
    this.m_led = led; 
    addRequirements(m_turret,led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.isFollowingTarget = true;
    m_led.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Valid : " + Robot.isValidAngle());
    if (Robot.isValidAngle()) {
        yaw = Robot.getVisionYawAngle();
        error = goal - yaw;

        if (error < 0) {
            m_turret.runTurret(0.3);
            side = "l";
            if (error > -10) {
                m_turret.runTurret(0.2);

            }
        } else if (error > 0) {
            side = "r";

            m_turret.runTurret(-0.3);
            if (error < 10) {
                m_turret.runTurret(-0.2);
            }
        } else {
            m_turret.runTurret(0);
        }
        if(m_turret.HeadEncoderGetValue() <= -5.8){
            if(side == "r"){
                m_turret.runTurret(0);
            }

        }

        if(m_turret.HeadEncoderGetValue() >= 5.8){
            if(side == "l"){
                m_turret.runTurret(0);
            }

        }
        if(error >= -2 && error <= 2){
            m_turret.isAtSetpoint = true;
            m_turret.runTurret(0);
          }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.runTurret(0);
    m_turret.isFollowingTarget = false;
    m_led.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
