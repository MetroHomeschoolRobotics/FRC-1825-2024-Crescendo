// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ShootToSpeaker extends Command {
  private Shooter shooter;
  private Wrist wrist;
  private double timer;

  private double kp = 0.02;
  private PIDController anglePID = new PIDController(kp, 0, 0);
  private PIDController anglePID2 = new PIDController(0.005, 0, 0);

  /** Creates a new ShootToSpeaker. */
  public ShootToSpeaker(Shooter _shooter, Wrist _wrist) {
    addRequirements(_shooter, _wrist);
    wrist = _wrist;
    shooter = _shooter;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = anglePID.calculate(wrist.getAbsoluteAngle(), MathUtil.clamp(shooter.getAngleToSpeaker(), -60, 60));
  
    wrist.setSpeed(setpoint);
    shooter.setSpeed(1);

    if (shooter.getSpeedShooter1() >= 5000 && shooter.getSpeedShooter2() >= 5000 && anglePID.atSetpoint() || timer >= 2) {
    // if (shooter.getSpeedShooter1() >= 200 && shooter.getSpeedShooter2() >= 200 && anglePID.atSetpoint() || timer >= 2) {
      shooter.setIndexerSpeed(0.3);
    }

    if (!shooter.noteInShooter()) {
      double setpoint2 = anglePID2.calculate(wrist.getAbsoluteAngle(), 60);
      wrist.setSpeed(setpoint2);
    }

    timer += 0.04;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    wrist.setSpeed(0);
    shooter.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return anglePID2.atSetpoint();
  }
}
