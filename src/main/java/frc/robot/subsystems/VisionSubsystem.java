// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  
  // Change this to match the name of your camera
  PhotonCamera camera;
  PhotonPipelineResult result;
  PhotonTrackedTarget target;
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);  
  
  public VisionSubsystem() {
    camera = new PhotonCamera("photonvision");
  }

  public PhotonPipelineResult getLatestResult(){
    var result = camera.getLatestResult();
    return result;
  }

  public PhotonTrackedTarget getBestTarget(){
    var target = result.getBestTarget();
    return target;
  }

  public void setDriverMode(){
    camera.setDriverMode(true);
  } 

  public void setVisionMode(){
    camera.setPipelineIndex(2);
  }

  public boolean hasTargets(){
    boolean hasTargets = result.hasTargets();
    return hasTargets;
  }

  public double getYaw(){
    double yaw = target.getYaw();
    return yaw;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
