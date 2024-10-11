// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class RunShooter extends Command {

  private Shooter shooter;
  private double timer;
  private Wrist wrist;

  /** Creates a new RunShooter. */
  public RunShooter(Shooter _shooter, Wrist _wrist) {
    addRequirements(_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = _shooter;
    wrist = _wrist;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.noteInShooter()) {
      shooter.setSpeed(1);

      if (shooter.getSpeedShooter1() >= 2000 /*&& shooter.getSpeedShooter2() >= 5000*/ || timer >= 1.5) {
        shooter.setIndexerSpeed(0.3);
      }
    }

    timer += 0.04;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    shooter.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.noteInShooter();
  }
}
