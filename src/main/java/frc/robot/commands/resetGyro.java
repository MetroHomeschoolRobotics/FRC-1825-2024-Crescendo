// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class resetGyro extends Command {
 // private double newHeading;
 private SwerveSubsystem drive;
  /** Creates a new resetGyro. */
  public resetGyro(SwerveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     Optional<Alliance> ally = DriverStation.getAlliance();
     if (ally.get() == Alliance.Red) {
      drive.setGyro(Math.PI);
     }
     else if (ally.get() == Alliance.Blue) {
      drive.setGyro(0);
     }
     else{
     System.out.println("No Alliance color");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
