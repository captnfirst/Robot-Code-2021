// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class BorusanTrajectory {
    DriveSubsystem m_drive;
    public Trajectory[] testAuto = new Trajectory[2];

    public BorusanTrajectory(DriveSubsystem drive) {
        m_drive = drive;
    }

}
