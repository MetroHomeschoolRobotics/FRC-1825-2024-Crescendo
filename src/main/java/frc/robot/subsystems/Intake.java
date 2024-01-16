// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public double getDistance() {
    return intakeMotor.getEncoder().getPosition();
  }

  public double getSpeed() {
    return intakeMotor.getEncoder().getVelocity();
  }
}
