// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.GeometryUtil;

public class RunElevator extends Command {

  private Elevator elevator;
  private Wrist wrist;

  private CommandXboxController xboxcontroller;
  /** Creates a new RunElevator. */
  public RunElevator(Elevator _elevator, Wrist _wrist, CommandXboxController _xboxController) {
    addRequirements(_elevator);

    elevator = _elevator;
    wrist = _wrist;
    xboxcontroller = _xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Distance To Limit", GeometryUtil.distanceToLimit(Math.abs(elevator.getDistance())/10.13, wrist.getAbsoluteAngle()));

    elevator.setSpeed(MathUtil.applyDeadband(xboxcontroller.getRightY(), 0.03), GeometryUtil.distanceToLimit(Math.abs(elevator.getDistance())/11.47, wrist.getAbsoluteAngle()));
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
