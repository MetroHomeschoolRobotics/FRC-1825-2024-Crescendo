// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.URI;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OrangePiTagTracking;

public class RunTagTracking extends Command {

  //private AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout("C:/Users/Blah/Desktop/Github Folder/2024 Offseason Swerve Experiments/FRC-1825-2024-Crescendo/src/main/deploy/2024-crescendo-apriltagposes.json");

  private OrangePiTagTracking tagTracking;

  /** Creates a new RunTagTracking. */
  public RunTagTracking() {
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //estimateFieldToRobotAprilTag(tagTracking.getCameraToTarget(), tagTracking.fieldRelativeTagPose, tagTracking.cameraToRobot);
  }
  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
