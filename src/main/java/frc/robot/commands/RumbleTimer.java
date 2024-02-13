// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleTimer extends Command {
  /** Creates a new RumbleTimer. */

  private CommandXboxController xboxController;

  public RumbleTimer(CommandXboxController xboxController_) {
    // Use addRequirements() here to declare subsystem dependencies.
    xboxController = xboxController_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getMatchTime() >=5 && DriverStation.getMatchTime() <= 7) { // Was 130 sec. (1 min. and 10 sec.)
      System.out.println("I work!!!");
      xboxController.getHID().setRumble(RumbleType.kBothRumble, 1);
    } else {
      System.out.println("I don't work!!!");
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
