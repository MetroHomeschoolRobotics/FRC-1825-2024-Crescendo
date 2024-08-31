// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class SetWristToAngle extends Command {

  private Wrist wrist;
  private PIDController anglePID = new PIDController(0.008, 0, 0);

  private double angle;
  private double timer;
  private double speed;

  /** Creates a new SetWristToAngle. */
  // This version of the constructor is used if the speed is unspecified, it is assumed to be 1
  public SetWristToAngle(Wrist _wrist, double _angle) {
    addRequirements(_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = _wrist;
    angle = _angle;
    speed = 1.0;
  }
  /** Creates a new SetWristToAngle. */
  // This version of the constructor is used if the speed is specified when the function is called
  public SetWristToAngle(Wrist _wrist, double _angle, double _speed) {
    addRequirements(_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = _wrist;
    angle = _angle;
    speed = _speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    anglePID.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = anglePID.calculate(wrist.getAbsoluteAngle(), angle);
    wrist.setSpeed(setpoint * speed, 100);
    timer += 0.04;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setSpeed(0, 100);
    // System.out.println("part 2 ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return anglePID.atSetpoint() || timer > 1;
  }
}
