// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  NetworkTableEntry rawBytesEntry;
  PhotonCamera camera = new PhotonCamera("LifeCam");
  PhotonPipelineResult result;
  NetworkTableEntry yawResult;
  NetworkTableInstance NTmain;
  NetworkTable nt; 
  public Vision() {
    camera = new PhotonCamera("LifeCam");
    NTmain = NetworkTableInstance.getDefault();
    nt = NTmain.getTable("photonvision").getSubTable("LifeCam");
  }

  /*
  public double getYaw(){
    if(result.hasTargets()){
      return result.getBestTarget().getYaw();
    }
    else return 0;
  }
  */
  
  public PhotonPipelineResult getLatestResult() {
    var ret = new PhotonPipelineResult();
    return ret;
  }
  /*
  public boolean hasTargets() {
      return getLatestResult().hasTargets();
  }*/
  
  public double getYaw(){
    return nt.getEntry("targetYaw").getDouble(Double.NaN);
  }
  

  public boolean hasTarget(){
    return nt.getEntry("hasTargets").getBoolean(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    SmartDashboard.putNumber("yawSD", getYaw());
  }
}
