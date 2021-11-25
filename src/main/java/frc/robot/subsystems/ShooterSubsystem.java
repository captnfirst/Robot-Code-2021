// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  PWMVictorSPX ShooterMotor;
  public boolean isAtSetpoint = false;
  int i;
  double rpmSum;
  public ShooterSubsystem() {
    ShooterMotor = new PWMVictorSPX(ShooterConstants.kShooterMotor);
    ShooterMotor.setInverted(ShooterConstants.kShooterInvertedMode1);
    ShooterMotor.setSafetyEnabled(false);
  }

  public void runShooter(double m_speed){
		ShooterMotor.set(m_speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
