// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private CANSparkMax elevatorMotor1 = new CANSparkMax(Constants.elevatorMotorID1, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax elevatorMotor2 = new CANSparkMax(Constants.elevatorMotorID2, CANSparkLowLevel.MotorType.kBrushless);
  private DigitalInput beamBreak = new DigitalInput(1);


  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor1.setInverted(true);
  }

  public void setSpeed(double speed, double distanceToLimit ) {
    if(!beamBreak.get() || speed <= 0 || getDistance() >= -195 || distanceToLimit < 5) {
      elevatorMotor1.set(speed);
      elevatorMotor2.set(speed);
    }
  }
  public double getDistance() {
    return (elevatorMotor1.getEncoder().getPosition()+elevatorMotor2.getEncoder().getPosition()/2);
  }
  public void resetEncoders() {
    elevatorMotor1.getEncoder().setPosition(0);
    elevatorMotor2.getEncoder().setPosition(0);
  }


  public Boolean isLowest() {
    return beamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elevator is lowest", isLowest());
    SmartDashboard.putNumber("elevator Distance", getDistance());

    if(isLowest()) {
      resetEncoders();
    }
    // This method will be called once per scheduler run
  }
}
