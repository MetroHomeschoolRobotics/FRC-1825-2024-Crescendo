package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class Teleop extends Command {
  private CommandXboxController xboxController;
  
  private Drivetrain drivetrain;

  private double deadband = 0.03;

  private double drivePeriod_;
  
  public Teleop(CommandXboxController xboxController_, Drivetrain drivetrain_, double drivePeriod) {
    addRequirements(drivetrain_);
    xboxController = xboxController_;
    drivetrain = drivetrain_;
    drivePeriod_ = drivePeriod;
  }
  
  public void initialize() {
    drivetrain.resetGyro();
  }

  public void execute() {
    //drivetrain.translateSpin(MathUtil.applyDeadband(xboxController.getLeftX(), deadband) , MathUtil.applyDeadband(-xboxController.getLeftY(), deadband), MathUtil.applyDeadband(xboxController.getRightX(), deadband));
    drivetrain.driveModules(xboxController.getLeftX(), xboxController.getLeftY(), xboxController.getRightX(), false, drivePeriod_);
  }
  
  public void end(boolean interrupted) {}
  
  public boolean isFinished() {
    return false;
  }
}
