// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class AimAtAmp extends Command {

  private Wrist wrist;
  private PIDController anglePID = new PIDController(0.02, 0, 0);
  private Shooter shooter;
  private Elevator elevator;
  private double timer;

  /** Creates a new AimAtAmp. */
  public AimAtAmp(Wrist _wrist, Shooter _shooter, Elevator _elevator) {
    addRequirements(_wrist);
    addRequirements(_shooter);
    addRequirements(_elevator);

    wrist = _wrist;
    shooter = _shooter;
    elevator = _elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.enableContinuousInput(-180, 180);
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = anglePID.calculate(wrist.getAbsoluteAngle(), -35);

    double shooterSpeed = 0.2;
    if (anglePID.atSetpoint() || timer > 2) {
      shooter.setIndexerSpeed(0.3);
    }else {
      elevator.setSpeed(-0.15, 13);
    }
    shooter.setSpeed(shooterSpeed);
    wrist.setSpeed(speed);

    timer += 0.04;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setSpeed(0);
    shooter.setSpeed(0);
    shooter.setIndexerSpeed(0);
    elevator.setSpeed(0, 13);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.noteInShooter();
  }
}