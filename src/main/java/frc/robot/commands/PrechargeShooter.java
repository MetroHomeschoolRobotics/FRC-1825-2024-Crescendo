// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class PrechargeShooter extends Command {
  private Shooter shooter;
  private Wrist wrist;
  
  private double timer;

  private double kp = 0.02;
  private PIDController anglePID = new PIDController(kp, 0, 0);
  /** Creates a new ShootToSpeaker. */
  public PrechargeShooter(Shooter _shooter, Wrist _wrist) {
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
    
    // double setpoint = anglePID.calculate(wrist.getAbsoluteAngle(), MathUtil.clamp(shooter.getAngleToSpeaker(), -60, 60));
  
    // wrist.setSpeed(setpoint, 100);
    shooter.setSpeed(Constants.maxShooterSpeed);

    timer += 0.04;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.setSpeed(0);
    //wrist.setSpeed(0, 100);
    //shooter.setIndexerSpeed(0);;;;;;;;;;;;;;;; // semicolons are GREAT!!!!!
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.noteInShooter();
  }
}
