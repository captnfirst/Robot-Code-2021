// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPIDCommand extends CommandBase {
  /** Creates a new TurretPIDCommand. */
  private TurretSubsystem m_turret;
  private LedSubsystem m_led;

  double error;
  double output;
  double outputSum;
  char lookingSide;

  public TurretPIDCommand(TurretSubsystem turret, LedSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_turret = turret;
    this.m_led = led;
    addRequirements(m_turret,led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    output = 0;
    m_led.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = 0;
    error = 0;
    if (Robot.isValidAngle()) {
      error = Robot.getVisionYawAngle();
      output = error * TurretConstants.kP;

      if (output > 6) {
          output = 6;
      } else if (output < -6) {
        output = -6;
      }
      if (m_turret.HeadEncoderGetValue() <= -5.8 ) {
          output = 0;
      }

      if (m_turret.HeadEncoderGetValue() >= 5.8) {
        output = 0;
      }

      if (error >= -2 && error <= 2) {
        m_turret.isAtSetpoint = true;
        // m_turret.isFollowingTarget = false; // might have to remove this
        output = 0;
      } else {
        m_turret.isAtSetpoint = false;
      }
      outputSum += output;
    } else {
        output = 0;
    }
    if (0 < output && 2 > output) {
      output += 1;
    } else if (0 > output && -2 < output) {
      output -= 1;
    }
    m_turret.runTurret(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.runTurret(0);
    m_led.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
