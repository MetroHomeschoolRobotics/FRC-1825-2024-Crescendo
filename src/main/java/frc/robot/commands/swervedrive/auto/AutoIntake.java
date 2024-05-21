// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class AutoIntake extends Command {

  private Intake intake;
  private Shooter indexer;
  private Wrist wrist;

  private double timer;

  /** Creates a new RunIntake. */
  public AutoIntake(Intake _intake, Shooter _indexer, Wrist _wrist) {
    addRequirements(_intake);
    addRequirements(_indexer);
    addRequirements(_wrist);
    
    intake = _intake;
    indexer = _indexer;
    wrist = _wrist;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;;;;;;;;;;;;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if (!indexer.noteInShooter()) {
        if(wrist.getAbsoluteAngle() >= 59) {
          intake.setSpeed(1);
          indexer.setIndexerSpeed(0.3);
        }

      }else {
        intake.setSpeed(0);
        indexer.setIndexerSpeed(0);

      }

      timer+=0.04;    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    intake.setSpeed(0);
    indexer.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return indexer.noteInShooter() || timer >= 4;
  }
}
