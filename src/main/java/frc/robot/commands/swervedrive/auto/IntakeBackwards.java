// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class IntakeBackwards extends Command {

  private Intake intake;
  private SwerveSubsystem drivetrain;
  private Wrist wrist;
  private Shooter indexer;
  private double timer;

  /** Creates a new IntakeBackwards. */
  public IntakeBackwards(Intake _intake, SwerveSubsystem swerveSubsystem, Wrist _wrist, Shooter _indexer) {
    addRequirements(_intake, swerveSubsystem, _wrist, _indexer);

    intake = _intake;
    drivetrain = swerveSubsystem;
    wrist = _wrist;
    indexer = _indexer;
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
    if (!indexer.noteInShooter()) {
      drivetrain.setChassisSpeeds(new ChassisSpeeds(1,0,0));
      
      if(wrist.getAbsoluteAngle() >= 59) {
        intake.setSpeed(1);
        indexer.setIndexerSpeed(0.3);

      }
    timer += 0.04;

    }else {
      intake.setSpeed(0);
      indexer.setIndexerSpeed(0);
      drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));

    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0);
    indexer.setIndexerSpeed(0);
    drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 3 || indexer.noteInShooter();
  }
}
