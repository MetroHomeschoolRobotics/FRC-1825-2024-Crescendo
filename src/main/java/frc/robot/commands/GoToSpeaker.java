// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.utils.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.config.NTData;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.aim.AimCalculator;

public class GoToSpeaker extends Command {
  private final SwerveSubsystem drivetrain;
  private final Shooter shooter;
  private PIDController pid;
  private double errorRad;
  private Rotation2d angleToTargetChanged;

  /** Creates a new GoToSpeaker. */
  public GoToSpeaker(SwerveSubsystem _drivetrain, Shooter _shooter) {
    drivetrain = _drivetrain;
    shooter = _shooter;
    addRequirements(drivetrain, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getPose();
    ChassisSpeeds robotSpeeds = drivetrain.getRobotVelocity();

    Translation2d robotPos = robotPose.getTranslation();
    Translation2d target = shooter.getSpeakerPosition();
    
    double distToTarget = target.getDistance(robotPos);
    Rotation2d angleToTarget = target.minus(robotPos).getAngle();
    double angleToTargetDeg = angleToTarget.getDegrees()+180;
    angleToTargetChanged = new Rotation2d(Units.degreesToRadians(angleToTargetDeg));
    // just simple turn to target
    drivetrain.drive(drivetrain.getTargetSpeeds(0, 0, angleToTargetChanged));

// TODO Implement Shooter logic 
        // AimCalculator.Aim aim = shooter.getTargetAim();
        // double horizFlywheelVelocity = aim.flywheelVelocity() * Math.cos(aim.pivotAngle());
        // double horizNoteVelocity = horizFlywheelVelocity
        //         / NTData.SHOOTER_MOVING_FLYWHEEL_VELOCITY.get()
        //         * NTData.SHOOTER_MOVING_EXIT_VELOCITY.get();
        // double flightTime = distToTarget / horizNoteVelocity;
        // double tangentialVel = -(angleToTarget.getSin() * robotSpeeds.vxMetersPerSecond + angleToTarget.getCos() * robotSpeeds.vyMetersPerSecond);
        // double correctionRad = 0 * Math.atan2(tangentialVel * flightTime, distToTarget);
        // System.out.printf("FH: %.3f NH: %.3f D: %.3f FT: %.3f TV: %.3f C(d): %.3f\n", horizFlywheelVelocity, horizNoteVelocity, distToTarget, flightTime, tangentialVel, Math.toDegrees(correctionRad));

        // double setpointAngle = MathUtil.wrap(angleToTarget.getRadians() + correctionRad, -Math.PI, Math.PI);
        // double currentAngle = MathUtil.wrap(robotPose.getRotation().getRadians(), -Math.PI, Math.PI);

        // double max = NTData.DRIVE_AIM_MAX_TURN.get() * MathUtil.TAU;
        // double output = pid.calculate(currentAngle, setpointAngle);
        // errorRad = pid.getPositionError();
        // output = MathUtil.clamp(output, -max, max);

    }

    public boolean isInTolerance(double tolRotations) {
        return errorRad < tolRotations * MathUtil.TAU;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleToTargetChanged.getDegrees() <= 2 && angleToTargetChanged.getDegrees() >= -2;
  }
}
