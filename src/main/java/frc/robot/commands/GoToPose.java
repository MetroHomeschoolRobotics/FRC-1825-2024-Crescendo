// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrajectorySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class GoToPose extends Command {

  private SwerveSubsystem swerveSubsystem;
  private TrajectorySubsystem trajectorySub;

  private double startVelocity;

  private Timer timer = new Timer();
  private Pose2d endPose;
  private Trajectory trajectory;

  /** Creates a new GoToPose. */
  public GoToPose(SwerveSubsystem _swerveSubsystem, TrajectorySubsystem _trajectorySub, double _startVelocity, Pose2d _endPose) {
    swerveSubsystem = _swerveSubsystem;
    trajectorySub = _trajectorySub;

    startVelocity = _startVelocity;
    endPose = _endPose;

    addRequirements(_trajectorySub, _swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    trajectorySub.setStartVelocity(startVelocity);
    trajectorySub.setEndVelocity(0);

    // translations.add(endPose.getTranslation());

    timer.start();
    trajectory = trajectorySub.generateTrajectory(swerveSubsystem.getPose(), List.of(endPose.getTranslation()), endPose);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ChassisSpeeds speeds = trajectorySub.getResultingSpeeds(trajectory, endPose, timer.get());

    swerveSubsystem.drive(speeds);

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
