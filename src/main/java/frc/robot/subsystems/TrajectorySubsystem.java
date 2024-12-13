// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrajectorySubsystem extends SubsystemBase {

  private TrajectoryConfig config = new TrajectoryConfig(Constants.maxSpeedMPerSec, Constants.maxAccelMPerSec);

  private HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(5, 0, 0), 
    new PIDController(5, 0, 0), 
    new ProfiledPIDController(5, 0, 0, 
        new TrapezoidProfile.Constraints(Units.degreesToRadians(760), Units.degreesToRadians(700))));

  /** Creates a new TrajectorySubsystem. */
  public TrajectorySubsystem() {}

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html 
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html


  public void setStartVelocity(double startVelocity) {
    config.setStartVelocity(startVelocity);
  }
  public void setEndVelocity(double endVelocity) {
    config.setEndVelocity(endVelocity);
  }

  public Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> innerWaypoints, Pose2d endPose) {

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, innerWaypoints, endPose, config);

    return trajectory;
  }

  public ChassisSpeeds getResultingSpeeds(Trajectory trajectory, Pose2d endPose, double timestampSec) {


    ChassisSpeeds speeds = controller.calculate(trajectory.getInitialPose(), trajectory.sample(timestampSec), endPose.getRotation());

    return speeds;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
