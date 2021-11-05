// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.borusanlib.util.LED;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */
  public LED led = new LED(0);
  public LedSubsystem() {}

  public void turnOn(){
    led.turnOn();
  }

  public void turnOff(){
    led.turnOff();
  }

  public void toggle(){
    led.toggle();
  }
  public boolean getState(){
    return led.get();
  }
}
