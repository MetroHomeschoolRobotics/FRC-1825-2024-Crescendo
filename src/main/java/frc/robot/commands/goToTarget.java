// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OrangePiTagTracking;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class goToTarget extends Command {

  private PIDController turnPID = new PIDController(2.5, 0, 0);
  private SwerveSubsystem swerveSubsystem;
  private OrangePiTagTracking orangePiTagTracking;
  double linearDistance;

  /** Creates a new goToTarget. */
  public goToTarget(SwerveSubsystem _SwerveSubsystem, OrangePiTagTracking _OrangePiTagTracking) {

    swerveSubsystem = _SwerveSubsystem;
    orangePiTagTracking = _OrangePiTagTracking;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_SwerveSubsystem, _OrangePiTagTracking);
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (orangePiTagTracking.hasTargets()) {
      linearDistance = orangePiTagTracking.getLinearDistanceFromTarget();

      double turnOutput = turnPID.calculate(Units.degreesToRadians(orangePiTagTracking.getYaw()), 0);

      swerveSubsystem.drive(new Translation2d(), turnOutput, false);
    }
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
