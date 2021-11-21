// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PushConstants;

public class PushSubsystem extends SubsystemBase {
  /** Creates a new PushSubsystem. */
  PWMVictorSPX PushMotor;
  public PushSubsystem() {
    PushMotor = new PWMVictorSPX(PushConstants.kPushMotor);
    PushMotor.setSafetyEnabled(false);
  }

  public void runPush(double m_speed){
		PushMotor.set(m_speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}