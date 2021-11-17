// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmIntakeSubsystem extends SubsystemBase {
  /** Creates a new ArmIntakeSubsystem. */
  
  PWMVictorSPX ArmIntakeMotor;

  public ArmIntakeSubsystem() {
    ArmIntakeMotor = new PWMVictorSPX(Constants.IntakeConstants.kArmIntakeMotor);
    ArmIntakeMotor.setSafetyEnabled(false);
  }

  public void runArmIntake(double m_speed){
		ArmIntakeMotor.set(m_speed);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}