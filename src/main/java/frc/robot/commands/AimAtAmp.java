// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class AimAtAmp extends Command {

  private Wrist wrist;
  private PIDController anglePID = new PIDController(0.01, 0, 0);
  private Shooter shooter;

  /** Creates a new AimAtAmp. */
  public AimAtAmp(Wrist _wrist, Shooter _shooter) {
    addRequirements(_wrist);
    addRequirements(_shooter);

    wrist = _wrist;
    shooter = _shooter;
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
    double speed = -anglePID.calculate(wrist.getAbsoluteAngle(), 120);
    double shooterSpeed = 0.5;
    
    if (shooter.getSpeedShooter1() >= 2000 && shooter.getSpeedShooter2() >= 2000) {
      shooter.setIndexerSpeed(0.3);
    }
    shooter.setSpeed(shooterSpeed);

    wrist.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setSpeed(0);
    shooter.setSpeed(0);
    shooter.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return anglePID.atSetpoint();
  }
}