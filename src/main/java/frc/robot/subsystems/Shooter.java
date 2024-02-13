// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterMotor = new CANSparkMax(Constants.shooterMotorID, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public double getDistance() {
    return shooterMotor.getEncoder().getPosition();
  }

  public double getSpeed() {
    return shooterMotor.getEncoder().getVelocity();
  }
}
