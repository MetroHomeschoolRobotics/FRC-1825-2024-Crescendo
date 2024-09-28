// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.GeometryUtil;

public class RunWrist extends Command {
  private Wrist wrist;
  private Intake intake;
  private Elevator elevator;
    private CommandXboxController xboxcontroller;

  /** Creates a new RunWrist. */
  public RunWrist(Wrist _wrist, Intake _intake, Elevator _elevator, CommandXboxController _xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_wrist);
    addRequirements(_intake);

    wrist = _wrist;
    intake = _intake;
    elevator = _elevator;
    xboxcontroller = _xboxController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intake.noteInIntake()) {
       wrist.setSpeed(MathUtil.applyDeadband(xboxcontroller.getLeftY(),0.03), GeometryUtil.distanceToLimit(Math.abs(elevator.getDistance())*(18/195.0), wrist.getAbsoluteAngle()));
    }
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