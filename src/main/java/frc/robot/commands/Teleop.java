package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class Teleop extends Command {
  private CommandXboxController xboxController;
  
  private Drivetrain drivetrain;

  private double deadband = 0.05;
  private double drivePeriod_;
  private Boolean boost = false;

  private double xSpeed;
  private double ySpeed;
  private double rotation;
  
  public Teleop(CommandXboxController xboxController_, Drivetrain drivetrain_, double drivePeriod) {
    addRequirements(drivetrain_);
    xboxController = xboxController_;
    drivetrain = drivetrain_;
    drivePeriod_ = drivePeriod;
  }
  
  public void initialize() {
    drivetrain.resetGyro();
    drivetrain.resetDistance();
  }

  public void execute() {
    xSpeed = Math.pow(MathUtil.applyDeadband(-xboxController.getLeftX(), deadband), 3);
    ySpeed = Math.pow(MathUtil.applyDeadband(-xboxController.getLeftY(), deadband), 3);
    rotation = Math.pow(MathUtil.applyDeadband(xboxController.getRightX(), deadband), 3);
    boost = xboxController.a().getAsBoolean() == true;
    
    //drivetrain.spinModuleVolts();
    // drivetrain.translateSpin(xSpeed , ySpeed, rotation, boost);
    drivetrain.driveModules(xSpeed , ySpeed, rotation, true, drivePeriod_);
    // drivetrain.frontRightMod.angleMotor.setVoltage(12*xboxController.getLeftY());
    // SmartDashboard.putNumber("Voltage", 12*xboxController.getLeftY());
    
    if (DriverStation.getMatchTime() >=5 && DriverStation.getMatchTime() <= 7) { // Was 130 sec. (1 min. and 10 sec.)
      xboxController.getHID().setRumble(RumbleType.kBothRumble, 1);
    } else {
      xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }
  
  public void end(boolean interrupted) {}
  
  public boolean isFinished() {
    return false;
  }
}
