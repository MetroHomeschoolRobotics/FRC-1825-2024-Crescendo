// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static final class swerveConstants {
    public static final class swerveModuleFR {
      public static final int angleMotorID = 6;
      
      public static final int angleEncoderID = 3;
      
      public static final int driveMotorID = 5;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;
    }
    public static final class swerveModuleFL {
      public static final int angleMotorID = 4;
      
      public static final int angleEncoderID = 2;
      
      public static final int driveMotorID = 3;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;
    }
    public static final class swerveModuleBR {
      public static final int angleMotorID = 8;
      
      public static final int angleEncoderID = 4;
      
      public static final int driveMotorID = 7;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;
    }
    public static final class swerveModuleBL {
      public static final int angleMotorID = 2;
      
      public static final int angleEncoderID = 1;
      
      public static final int driveMotorID = 1;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;
    }
  }
  
  public static final class autoConstants {
    public static final double kpDriveVelocity = 0;
    public static final double kpTurnVelocity = 0;

    // front vs back
    public static final double trackWidth = Units.inchesToMeters(20+(7/16));
    // left vs right
    public static final double trackHeight = Units.inchesToMeters(20+(7/16));
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(trackHeight/2, -trackWidth/2),
      new Translation2d(trackHeight/2, trackWidth/2), 
      new Translation2d(-trackHeight/2, -trackWidth/2), 
      new Translation2d(-trackHeight/2, trackWidth/2));
    public static final TrapezoidProfile.Constraints spinPIDConstraints = new TrapezoidProfile.Constraints((180), 360);

    public static final double maxSpeedMetersPerSecond = 0;
  }

  public static final int intakeMotorID = 9;
}
