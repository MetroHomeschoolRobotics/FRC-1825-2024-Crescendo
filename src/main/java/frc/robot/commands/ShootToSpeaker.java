// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.auto.ShootToAngle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ShootToSpeaker extends Command {
  private Shooter shooter;
  private Wrist wrist;

  /** Creates a new ShootToSpeaker. */
  public ShootToSpeaker(Shooter _shooter, Wrist _wrist) {
    addRequirements(_shooter, _wrist);
    shooter = _shooter;
    wrist = _wrist;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    new ShootToAngle(shooter, wrist, -2.5954*Math.pow(shooter.getSpeakerDistance(), 3) + 27.224*Math.pow(shooter.getSpeakerDistance(), 2) - 97.353*shooter.getSpeakerDistance() + 147.07);  //math is cool //134.36-21.66*Math.log(Units.metersToInches(shooter.getSpeakerDistance()) TODO Should Probably be Math.atan(shooter.getSpeakerDistance()/HeightOfShot) 
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
