package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class Teleop extends Command {
  private CommandXboxController xboxController;
  
  private Drivetrain drivetrain;

  private double deadband = 0.03;
  
  public Teleop(CommandXboxController xboxController_, Drivetrain drivetrain_) {
    addRequirements(drivetrain_);
    xboxController = xboxController_;
    drivetrain = drivetrain_;
  }
  
  public void initialize() {
    drivetrain.resetGyro();
  }

  public void execute() {
    drivetrain.translateSpin(MathUtil.applyDeadband(xboxController.getLeftX(), deadband) , MathUtil.applyDeadband(-xboxController.getLeftY(), deadband), MathUtil.applyDeadband(xboxController.getRightX(), deadband));
    //drivetrain.translate(xboxController.getLeftX(), xboxController.getLeftY());
  }
  
  public void end(boolean interrupted) {}
  
  public boolean isFinished() {
    return false;
  }
}
