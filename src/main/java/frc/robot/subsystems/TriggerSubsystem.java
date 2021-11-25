// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TriggerConstants;

public class TriggerSubsystem extends SubsystemBase {
  /** Creates a new TriggerSubsystem. */
  PWMVictorSPX TriggerMotor;
  public TriggerSubsystem() {
    TriggerMotor = new PWMVictorSPX(TriggerConstants.kTriggerMotor);
    TriggerMotor.setSafetyEnabled(false);
  }

  public void runTrigger(double m_speed){
		TriggerMotor.set(m_speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
