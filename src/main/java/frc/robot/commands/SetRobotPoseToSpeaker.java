package frc.robot.commands;


import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.field.FieldInfo;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetRobotPoseToSpeaker extends Command {
    
  private SwerveSubsystem drive;
  private CommandXboxController xboxcontroller;

  /** Creates a new RunWrist. */
  public SetRobotPoseToSpeaker(SwerveSubsystem drive, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
  
    this.drive = drive;
    this.xboxcontroller = xboxController;
    addRequirements(this.drive);
  

}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.resetOdometry(FieldInfo.CRESCENDO_2024.flipPoseForAlliance(new Pose2d(1.2446, 5.512, new Rotation2d(Math.PI))));

     Optional<Alliance> ally = DriverStation.getAlliance();
     if (ally.get() == Alliance.Red) {
       drive.setGyro(drive.getPose().getRotation().getRadians());
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
