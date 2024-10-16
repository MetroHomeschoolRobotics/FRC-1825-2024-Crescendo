// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrangePiTagTracking extends SubsystemBase {

  private PhotonCamera orangePi = new PhotonCamera("Arducam_OV9281_USB_Camera");

  /** Creates a new OrangePiTagTracking. */
  public OrangePiTagTracking() {}
  

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

public boolean hasTargets(){
  return orangePi.getLatestResult().hasTargets();
}

public List<PhotonTrackedTarget> getTargets(){
 return orangePi.getLatestResult().getTargets();
}

public PhotonTrackedTarget getBestTarget() {
  return orangePi.getLatestResult().getBestTarget();
}

}
