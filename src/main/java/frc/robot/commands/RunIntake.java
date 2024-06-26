// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class RunIntake extends Command {

  private Intake intake;
  private Boolean reversed;
  private Shooter indexer;
  private Wrist wrist;

  /** Creates a new RunIntake. */
  public RunIntake(Intake _intake, Boolean _reversed, Shooter _indexer, Wrist _wrist) {
    addRequirements(_intake);
    addRequirements(_indexer);
    addRequirements(_wrist);
    
    intake = _intake;
    reversed = _reversed;
    indexer = _indexer;
    wrist = _wrist;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(reversed){
      intake.setSpeed(-1);
      indexer.setIndexerSpeed(-0.5);
      
    }else{
      if (!indexer.noteInShooter()) {
        if(wrist.getAbsoluteAngle() >= 57) {
          intake.setSpeed(1);
          indexer.setIndexerSpeed(0.3);
        }

      }else {
        intake.setSpeed(0);
        indexer.setIndexerSpeed(0);

      }
    }
    
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
    if (!reversed)
      return indexer.noteInShooter();
    else 
      return false;
  }
}
