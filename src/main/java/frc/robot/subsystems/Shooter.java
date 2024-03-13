// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.field.FieldInfo;
import frc.robot.subsystems.aim.AimCalculator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Shooter extends SubsystemBase {
  private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));
  private CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotorID1, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotorID2, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorID, CANSparkLowLevel.MotorType.kBrushless);
  private DigitalInput beamBrake = new DigitalInput(2);
  private final SwerveSubsystem drivetrain;

  private AimCalculator.Aim targetAim; 

  /** Creates a new Shooter. */
  public Shooter(SwerveSubsystem _drivetrain) {
    indexerMotor.setInverted(true);
    drivetrain = _drivetrain;
  }

  public Translation2d getSpeakerPosition() {
    return FieldInfo.CRESCENDO_2024.flipPoseForAlliance(blueSpeakerPose).getTranslation();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beam Break Triggered Shooter", !beamBrake.get());
    double SpeakerDistance = getSpeakerPosition().getDistance(drivetrain.getEstimatedPose().getTranslation());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }

  public void setIndexerSpeed(double speed) {
    indexerMotor.set(speed);
  }

  public Boolean noteInShooter() {
    return !beamBrake.get();
  }

  public double getDistance() {
    return shooterMotor1.getEncoder().getPosition();
  }

  public double getSpeedShooter1() {
    return shooterMotor1.getEncoder().getVelocity();
  }
  public double getSpeedShooter2() {
    return shooterMotor2.getEncoder().getVelocity();
  }
  public AimCalculator.Aim getTargetAim() {
        return targetAim;
  }
}
