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
  TurretSubsystem m_turret;

  LedSubsystem m_led;

  double yaw;

  double goal;
  
  double error;

  public AutoAlign(TurretSubsystem turret, LedSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_turret = turret; 
    this.m_led = led; 
    addRequirements(m_turret,led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Value1" + Robot.isValidAngle());

    if(Robot.isValidAngle()){
    System.out.println("Value2" + Robot.isValidAngle());

      yaw = Robot.getVisionYawAngle();
      
      error = goal - yaw;

      if(error<0){
        m_turret.runTurret(-0.3);
        if(error>-10){
          m_turret.runTurret(-0.2);
        }
      }else if(error>0){
        m_turret.runTurret(0.3);
        if(error<10){
          m_turret.runTurret(0.2);
        }
      }else{
        m_turret.runTurret(0);
      }
    }
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
    if (Robot.isValidAngle()) return (Math.abs(Robot.getVisionYawAngle()) < 2);
    else return false;
  }
}
