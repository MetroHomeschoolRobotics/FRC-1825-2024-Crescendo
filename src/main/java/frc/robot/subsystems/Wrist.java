// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GeometryUtil;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristMotor = new CANSparkMax(Constants.wristMotorID, CANSparkLowLevel.MotorType.kBrushless);
  private DutyCycleEncoder rotationEncoder = new DutyCycleEncoder(3);

  /** Creates a new Wrist. */
  public Wrist() {}

  public void setSpeed(double speed) {
    if (getAbsoluteAngle() >= -59  && getAbsoluteAngle() <= 59){
      wristMotor.set(speed);
    } else if ( getAbsoluteAngle() < -59 && speed >= 0) {
      wristMotor.set(speed);
    } else if ( getAbsoluteAngle() > 59 && speed <= 0) {
      wristMotor.set(speed);
    } else {
      wristMotor.set(0);
    }

    // if(speed <= 0 || getAbsoluteAngle() <= 59){
    //   wristMotor.set(speed);        
    // }        
  }

  public double getAbsoluteAngle() {
    return rotationEncoder.getAbsolutePosition()*(360/1)-215.1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getAbsoluteAngle());
    // This method will be called once per scheduler run
  }
}
