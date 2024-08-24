// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.aim.AimCalculator;
import frc.robot.subsystems.aim.TableAimCalculator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.tagtracker.TagTrackerInput;
import frc.robot.lib.field.FieldInfo;

public class Shooter extends SubsystemBase {
  private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));
  private CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotorID1, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotorID2, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorID, CANSparkLowLevel.MotorType.kBrushless);
  private DigitalInput beamBrake = new DigitalInput(2);
  private SwerveSubsystem drivetrain;
  private AimCalculator.Aim targetAim; // Target aim is null if not currently aiming
  private AimCalculator aimCalculator;
  private final AimCalculator tableAimCalculator;
  private double SpeakerNonEstimatedDistance;
  private double wristTrim = 0.0;

  /** Creates a new Shooter. */
  public Shooter(SwerveSubsystem _drivetrain) {
    indexerMotor.setInverted(true);
    drivetrain = _drivetrain;

    tableAimCalculator = new TableAimCalculator();
    aimCalculator = tableAimCalculator;
    
  }

  public Translation2d getSpeakerPosition() {
    FieldInfo fieldInfo = FieldInfo.CRESCENDO_2024;
    return fieldInfo.flipPoseForAlliance(blueSpeakerPose).getTranslation();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note In Shooter", !beamBrake.get());
    //double SpeakerDistance = getSpeakerPosition().getDistance(drivetrain.getDrive().swerveDrivePoseEstimator.getEstimatedPosition().getTranslation());
    double SpeakerDistance = getSpeakerPosition().getDistance(getEstimatePose().getTranslation());
    AimCalculator.Aim aim = aimCalculator.calculateAim(SpeakerDistance);
    //SmartDashboard.putNumber("Distance April Tag", SpeakerDistance);
    SpeakerNonEstimatedDistance = getSpeakerPosition().getDistance(drivetrain.getPose().getTranslation());
    // SmartDashboard.putNumber("Distance to Speaker", SpeakerNonEstimatedDistance);
    // SmartDashboard.putNumber("Speaker angle", getAngleToSpeaker());
    targetAim = aim;
    // This method will be called once per scheduler run
  }
  private Pose2d getEstimatePose(){
    Pose2d lastPose = new Pose2d();
    double lastTimestamp = 0.0;
    List<TagTrackerInput.VisionUpdate> visionData = drivetrain.getTagTracker().getNewUpdates();
    for (TagTrackerInput.VisionUpdate visionUpdate : visionData) {
      if (visionUpdate.timestamp > lastTimestamp) {
        lastTimestamp = visionUpdate.timestamp;
        lastPose = visionUpdate.estPose;
      }
     }
     return lastPose;
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

  public Command incrementTrimCommand() {
    return this.runOnce(() -> wristTrim = wristTrim + 0.5);
  }
public Command decrementTrimCommand() {
    return this.runOnce(() -> wristTrim = wristTrim - 0.5);
  }



  public double getSpeakerDistance() {
    return SpeakerNonEstimatedDistance;
  }
  public double getAngleToSpeaker(){
    double angle = -2.5954*Math.pow(getSpeakerDistance(), 3) + 27.224*Math.pow(getSpeakerDistance(), 2) - 97.353*getSpeakerDistance() + 147.07+1.5;
    return Math.max(angle + wristTrim, 25);
  }
}