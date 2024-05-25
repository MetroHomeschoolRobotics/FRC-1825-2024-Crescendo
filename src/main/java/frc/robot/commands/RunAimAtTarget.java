// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RunAimAtTarget extends Command {

  private PhotonCamera camera;
  private SwerveSubsystem swerveSubsystem;
  private Intake intake;
  private Shooter indexer;

  private PIDController turnPid = new PIDController(0.025, 0, 0);


  double yaw = 0;
  double pidOutput = 0;

  /** Creates a new RunAimAtTarget. */
  public RunAimAtTarget(PhotonCamera _Camera, SwerveSubsystem _swerveSubsystem, Intake _intake, Shooter shooter) {
    addRequirements(_swerveSubsystem, _intake, shooter);

    camera = _Camera;
    swerveSubsystem = _swerveSubsystem;
    intake = _intake;
    indexer = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pidOutput = turnPid.calculate(camera.getLatestResult().getBestTarget().getYaw(), 0);
    // turnPid.enableContinuousInput(-180, 180);

    turnPid.setTolerance(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PhotonPipelineResult result = camera.getLatestResult();
    
    if (result.hasTargets()) {
      
        

      yaw = result.getBestTarget().getYaw();

      pidOutput = 0;
      double driveOutput = 0;

      double angleOutput = 0;

      if (yaw != 0) {
        driveOutput = Math.min(6/Math.abs(yaw),3);
      }
      else{
        driveOutput = 3;
      }
      if(!turnPid.atSetpoint()) {
        pidOutput = turnPid.calculate(result.getBestTarget().getYaw(), 0);
        // }else{
      //   driveOutput = 3;
       }
      if(result.getBestTarget().getPitch() <= -20) {
        swerveSubsystem.drive(new ChassisSpeeds(3, 0, 0));
      }
      else {
      swerveSubsystem.drive(new ChassisSpeeds(driveOutput, pidOutput, 0));
      }
      indexer.setIndexerSpeed(0.3);
      intake.setSpeed(1);
      
    }
    // else {
    //   swerveSubsystem.drive(new ChassisSpeeds(1, 0, 0));
    // }

    SmartDashboard.putNumber("Angle to Note", yaw);
    SmartDashboard.putBoolean("At setpoint", turnPid.atSetpoint());

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds(0,0, 0));
    indexer.setIndexerSpeed(0);
    intake.setSpeed(0);
    turnPid.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.noteInShooter(); //indexer.noteInShooter();
  }
}
