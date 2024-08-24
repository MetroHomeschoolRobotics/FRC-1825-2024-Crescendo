// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleCalculator extends SubsystemBase {
  /** Creates a new AngleCalculator. */
  public AngleCalculator() {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
  }

  public double getLinearDistance(Pose3d robotPose, Pose3d otherPose) {
    double[] robotXY = {robotPose.getX(), robotPose.getY()};
    double[] otherXY = {otherPose.getX(), robotPose.getY()};

    double xDiff = Math.max(robotXY[0], otherXY[0]) - Math.min(robotXY[0], otherXY[0]);
    double yDiff = Math.max(robotXY[1], otherXY[1]) - Math.min(robotXY[1], otherXY[1]);

    double linearDistance = Math.sqrt(Math.pow(xDiff, 2)+Math.pow(yDiff, 2));

    return linearDistance;
  }

  public double getAngleFromPose(Pose3d robotPose, Pose3d otherPose) {
    double[] robotXY = {robotPose.getX(), robotPose.getY()};
    double[] otherXY = {otherPose.getX(), robotPose.getY()};

    double xDiff = Math.max(robotXY[0], otherXY[0]) - Math.min(robotXY[0], otherXY[0]);
    double yDiff = Math.max(robotXY[1], otherXY[1]) - Math.min(robotXY[1], otherXY[1]);

    double angle = Math.atan2(yDiff, xDiff);

    return angle;
  }



}
