// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;



public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  PWMTalonSRX leftRearMotor = new PWMTalonSRX(Constants.DriveConstants.kLeftRearMotor);
  //PWMTalonSRX leftFrontMotor = new PWMTalonSRX(Constants.DriveConstants.kLeftFrontMotor);
  PWMTalonSRX rightRearMotor = new PWMTalonSRX(Constants.DriveConstants.kRightRearMotor);
  //PWMTalonSRX rightFrontMotor = new PWMTalonSRX(Constants.DriveConstants.kRightFrontMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftRearMotor, rightRearMotor);
  public final DifferentialDriveOdometry m_odometry;


  Gyro m_gyro = new ADXRS450_Gyro();
  
  double target;
  double error;
  double kP = 1;

  public DriveSubsystem() {
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    zeroHeading();
  }

  public void curvatureDrive(double x,double y){
		m_drive.arcadeDrive(x, y, true);
	}

  public void TankDrive(double left, double right){
    m_drive.tankDrive(left*1, right*1, true);
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  } 

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public double getHeadingCW() {
    // Not negating
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public double getTurnRateCW() {
    // Not negating
    return m_gyro.getRate();
  }

  public double getTarget() {
    return getHeading() + target;
  }

  public void setTarget(double val) {
    target = val;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /*
  public double Rotate(){
    error = 180 - m_gyro.getRate();
    m_drive.tankDrive(kP * error, kP * error);
    return error;
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
