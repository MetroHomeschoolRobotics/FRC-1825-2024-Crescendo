// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public static final int kManipulatorControllerPort = 1;
  }
  
  public static final class swerveConstants {
    public static final class swerveModuleBR {
      public static final int angleMotorID = 6;
      
      public static final int angleEncoderID = 3;
      
      public static final int driveMotorID = 5;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;

      public static final double angleOffset = -311.39; // all angle offsets are in degrees
    }
    public static final class swerveModuleFR {
      public static final int angleMotorID = 4;
      
      public static final int angleEncoderID = 2;
      
      public static final int driveMotorID = 3;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;

      public static final double angleOffset = -238.06;
    }
<<<<<<< HEAD
    public static final class swerveModuleBR {
=======
    public static final class swerveModuleBL {
>>>>>>> 7a5b822db7ba12d6d46892076e5c2684cf5d3e53
      public static final int angleMotorID = 7;
      
      public static final int angleEncoderID = 4;
      
      public static final int driveMotorID = 8;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;

      public static final double angleOffset = -181.58;
    }
    public static final class swerveModuleFL {
      public static final int angleMotorID = 2;
      
      public static final int angleEncoderID = 1;
      
      public static final int driveMotorID = 1;
      
      public static final Boolean angleMotorReversed = false;
      
      public static final Boolean driveMotorReversed = false;

      public static final double angleOffset = -250.22;
    }
  }
  
  public static final class autoConstants {
    // front vs back
    public static final double trackWidth = Units.inchesToMeters(20+(7/16));
    // left vs right
    public static final double trackHeight = Units.inchesToMeters(20+(7/16));
    // add it all into the kinematics
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(trackHeight/2, -trackWidth/2),
      new Translation2d(trackHeight/2, trackWidth/2), 
      new Translation2d(-trackHeight/2, -trackWidth/2), 
      new Translation2d(-trackHeight/2, trackWidth/2));
    // The spin constraints of the wheels
    public static final TrapezoidProfile.Constraints spinPIDConstraints = new TrapezoidProfile.Constraints(720, 720);

    // max stuffs
    public static final double maxSpeedMetersPerSecond = 4.9; // 4.8 m/s
    public static final double maxSpeedRadiansPerSecond = 3.66;
  }

  public static final int intakeMotorID = 10;
<<<<<<< HEAD
  public static final int elevatorMotorID1 = 11;
  public static final int elevatorMotorID2 = 12;
  public static final int indexerMotorID = 13;
  public static final int shooterMotorID1 = 14;
  public static final int shooterMotorID2 = 15;
  public static final int wristMotorID = 16;

=======
  public static final int shooterMotorID = 11;
>>>>>>> 7a5b822db7ba12d6d46892076e5c2684cf5d3e53
}
