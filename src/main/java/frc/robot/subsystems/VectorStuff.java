// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VectorStuff extends SubsystemBase {
  /** Creates a new VectorStuff. */
  public VectorStuff() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double[] vectorAddition(double vecLength1, double angleDeg1, double vecLength2, double angleDeg2) {
    double vecX1 = vecLength1 * Math.cos(angleDeg1*(Math.PI/180));
    double vecY1 = vecLength1 * Math.sin(angleDeg1*(Math.PI/180));

    double vecX2 = vecLength2 * Math.cos(angleDeg2*(Math.PI/180));
    double vecY2 = vecLength2 * Math.sin(angleDeg2*(Math.PI/180));

    double vecAddedX = vecX1+vecX2;
    double vecAddedY = vecY1+vecY2;
    
    double vecAngle = Math.atan2(vecAddedY, vecAddedX);
    double vecLength = Math.sqrt(Math.pow(vecAddedX, 2)+Math.pow(vecAddedY, 2));
    double[] resultantVector = {vecLength, vecAngle};

    return resultantVector;
  }
}
