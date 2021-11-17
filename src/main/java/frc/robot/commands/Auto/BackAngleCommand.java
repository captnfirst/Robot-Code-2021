// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BackAngleCommand extends CommandBase {
  /** Creates a new BackAngleCommand. */
  Timer zamanlayici = new Timer();
  DriveSubsystem m_drive;
  public BackAngleCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zamanlayici.reset();
    zamanlayici.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(zamanlayici.get() > 0 && zamanlayici.get() < 1){
      m_drive.TankDrive(-0.5, 0.5);
    } else{
      m_drive.TankDrive(0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
