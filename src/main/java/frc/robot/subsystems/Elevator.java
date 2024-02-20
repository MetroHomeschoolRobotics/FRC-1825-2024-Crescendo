// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private CANSparkMax elevatorMotor1 = new CANSparkMax(Constants.elevatorMotorID, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax elevatorMotor2 = new CANSparkMax(Constants.elevatorMotorID, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor2.setInverted(true);
  }

  public void setSpeed(double speed) {
    elevatorMotor1.set(speed);
    elevatorMotor2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
