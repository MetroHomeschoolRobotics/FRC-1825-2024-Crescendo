// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.URI;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.OrangePiTagTracking;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class RunTagTracking extends Command {

  private OrangePiTagTracking tagTracking;
  private PIDController anglePID = new PIDController(0.02, 0, 0);
  private Wrist wrist;
  private Shooter shooter;
  private boolean released;

  /** Creates a new RunTagTracking. */
  public RunTagTracking(boolean _released,OrangePiTagTracking _tagTracking, Shooter _shooter, Wrist _wrist) {
    addRequirements(_tagTracking);
    addRequirements(_shooter);
    addRequirements(_wrist);

    tagTracking = _tagTracking;
    shooter = _shooter;
    wrist = _wrist;
    released = _released;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.enableContinuousInput(-180, 180);
  }
  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {

    System.out.println("trigger 1");

    if(tagTracking.hasTargets()/* && (tagTracking.getTagID() == 3 || tagTracking.getTagID() == 4 || tagTracking.getTagID() == 7 || tagTracking.getTagID() == 8) */ ) {
      System.out.println("trigger 2");
      if(!released){
        System.out.println("trigger 3");
        double distance = tagTracking.getLinearDistanceFromTarget();
        double speed = anglePID.calculate(wrist.getAbsoluteAngle(), MathUtil.clamp(shooter.getAngleToSpeaker(distance), -60, 60));

        wrist.setSpeed(speed, 100);
        shooter.setSpeed(Constants.shooterMaxSpeed);

      } else {
        shooter.setSpeed(Constants.shooterMaxSpeed);
        shooter.setIndexerSpeed(0.3);

      }
    }
    



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    shooter.setIndexerSpeed(0);
    wrist.setSpeed(0, 100);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
