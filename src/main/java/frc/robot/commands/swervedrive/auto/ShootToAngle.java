// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ShootToAngle extends Command {

  private Shooter shooter;
  private Wrist wrist;
  private double timer;
  private PIDController anglePID = new PIDController(0.005, 0, 0);

  /** Creates a new ShootToAngle. */
  public ShootToAngle(Shooter _shooter, Wrist _wrist) {
    addRequirements(_shooter, _wrist);
    
    shooter = _shooter;
    wrist = _wrist;
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
    //if(shooter.noteInShooter()) {
      double setpoint = anglePID.calculate(wrist.getAbsoluteAngle(), 32);

      wrist.setSpeed(setpoint);
      shooter.setSpeed(1);

      if (shooter.getSpeedShooter1() >= 5000 && shooter.getSpeedShooter2() >= 5000 && anglePID.atSetpoint() || timer >= 2) {
        shooter.setIndexerSpeed(0.3);
      }
    //}

    timer += 0.04;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 3 || !shooter.noteInShooter();
  }
}
