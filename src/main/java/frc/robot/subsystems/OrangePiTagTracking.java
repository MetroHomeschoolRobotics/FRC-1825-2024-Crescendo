// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OrangePiTagTracking extends SubsystemBase {

  private PhotonCamera orangePi = new PhotonCamera("Arducam_OV9281_USB_Camera");

  private AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
    tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, orangePi, Constants.tagCameraPosition);

  /** Creates a new OrangePiTagTracking. */
  public OrangePiTagTracking() {}
  

  @Override
  public void periodic() {
    
    // photonPoseEstimator.
    

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

public double getYaw() {
  return getBestTarget().getYaw();
}

public double getPitch() {
  return getBestTarget().getPitch();
}

public double getArea() {
  return getBestTarget().getArea();
}

public double skew() {
  return getBestTarget().getSkew();
}

public Optional<EstimatedRobotPose> getEstimatedPose3d(Pose2d previousPose) {

  photonPoseEstimator.setReferencePose(previousPose);

  return photonPoseEstimator.update();
}

public Transform3d getCameraToTarget() {
  return getBestTarget().getBestCameraToTarget();
}

public double getLinearDistanceFromTarget() {
  // this uses the distance formula to get the linear distance from (x,y) difference
  double linearDistance = Math.sqrt(Math.pow(getCameraToTarget().getX(), 2) + Math.pow(getCameraToTarget().getY(), 2)); 
  
  return linearDistance;
}


public int targetID() {
  return getBestTarget().getFiducialId();
}
}
