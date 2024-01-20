package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.swerveConstants;

public class Drivetrain extends SubsystemBase {


  // this gets each swerve module so I won't have to get each motor and encoder individually
  private SwerveModule frontRightMod = new SwerveModule(
      "Front Right", 
      swerveConstants.swerveModuleFR.angleEncoderID, swerveConstants.swerveModuleFR.angleMotorID, 
      swerveConstants.swerveModuleFR.driveMotorID, 
      swerveConstants.swerveModuleFR.angleMotorReversed, swerveConstants.swerveModuleFR.driveMotorReversed,
      -221.39);
  
  private SwerveModule frontLeftMod = new SwerveModule(
      "Front Left", 
      swerveConstants.swerveModuleFL.angleEncoderID, swerveConstants.swerveModuleFL.angleMotorID, 
      swerveConstants.swerveModuleFL.driveMotorID,
      swerveConstants.swerveModuleFL.angleMotorReversed, swerveConstants.swerveModuleFL.driveMotorReversed,
    -148.06);
  
  private SwerveModule backRightMod = new SwerveModule(
      "Back Right", 
      swerveConstants.swerveModuleBR.angleEncoderID, swerveConstants.swerveModuleBR.angleMotorID, 
      swerveConstants.swerveModuleBR.driveMotorID,
      swerveConstants.swerveModuleBR.angleMotorReversed, swerveConstants.swerveModuleBR.driveMotorReversed,
      -91.58);
  
  private SwerveModule backLeftMod = new SwerveModule(
      "Back Left", 
      swerveConstants.swerveModuleBL.angleEncoderID, swerveConstants.swerveModuleBL.angleMotorID, 
      swerveConstants.swerveModuleBL.driveMotorID,
      swerveConstants.swerveModuleBL.angleMotorReversed, swerveConstants.swerveModuleBL.driveMotorReversed,
      -160.22);
  
  private AHRS gyro = new AHRS();

  private SlewRateLimiter accelLimiter = new SlewRateLimiter(.95);

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.autoConstants.swerveKinematics, gyro.getRotation2d(), getModulePositions());

  

  public Drivetrain() {
    gyro.reset();
  }


  public void periodic() {
    SmartDashboard.putNumber("Gyro rotation", getRotation());
  }
  
  /*
   * Gyro
   */
  public void resetGyro() {
    gyro.reset();
  }

  public double getRotation(){
    return gyro.getAngle();
  }


  /*
   * Auto Stuffs
   */
  
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {frontLeftMod.getModuleState(), frontRightMod.getModuleState(), backLeftMod.getModuleState(), backRightMod.getModuleState()};
    return states;
  }
  
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = {frontLeftMod.getModulePosition(), frontRightMod.getModulePosition(), backLeftMod.getModulePosition(), backRightMod.getModulePosition()};
    return modulePositions;
  }
  
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }


  /*
   * Drive the robot
   */
  // for autos
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.autoConstants.maxSpeedMetersPerSecond);

    frontLeftMod.setDesiredState(desiredStates[0]);
    frontRightMod.setDesiredState(desiredStates[1]);
    backLeftMod.setDesiredState(desiredStates[2]);
    backRightMod.setDesiredState(desiredStates[3]);
  }

  // for controllers
  public void driveModules(double translateX, double translateY, double rotationX, Boolean fieldOriented, double periodSeconds) {
    SwerveModuleState[] swerveModuleStates = Constants.autoConstants.swerveKinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        fieldOriented ?
          ChassisSpeeds.fromFieldRelativeSpeeds(translateY, translateX, rotationX, gyro.getRotation2d())
          : new ChassisSpeeds(translateY, translateX, rotationX),
        periodSeconds));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 3);

        frontLeftMod.setDesiredState(swerveModuleStates[0]);
        frontRightMod.setDesiredState(swerveModuleStates[1]);
        backLeftMod.setDesiredState(swerveModuleStates[2]);
        backRightMod.setDesiredState(swerveModuleStates[3]);

        // frontLeftMod.setDesiredState(new SwerveModuleState(1, new Rotation2d(0)));
        // frontRightMod.setDesiredState(new SwerveModuleState(1, new Rotation2d(0)));
        // backLeftMod.setDesiredState(new SwerveModuleState(1, new Rotation2d(0)));
        // backRightMod.setDesiredState(new SwerveModuleState(1, new Rotation2d(0)));
  }


  public void translateSpin(double speedX, double speedY, double turnX) {
    speedX = -speedX;


    // Get the translate angle and add the gyro angle to it
    double translateAngle = Math.atan2(speedY, speedX) * (180/Math.PI) - gyro.getAngle();
    double translateSpeed = Math.sqrt(Math.pow(speedX, 2) + Math.pow(speedY, 2));

    SmartDashboard.putNumber("Translate Angle", translateAngle);

    speedX = translateSpeed * Math.cos(translateAngle * (Math.PI/180));
    speedY = translateSpeed * Math.sin(translateAngle * (Math.PI/180));


    // This converts everything to vectors and adds them ☺
    double TurnVectorXM1 = turnX * Math.cos(Math.PI/4 + (Math.PI*3)/2 + Math.PI/2 + Math.PI); // front right 45 + 270
    double TurnVectorYM1 = turnX * Math.sin(Math.PI/4 + (Math.PI*3)/2 + Math.PI/2 + Math.PI);
    double TurnVectorXM2 = turnX * Math.cos(Math.PI/4 + Math.PI/2); // front left 45
    double TurnVectorYM2 = turnX * Math.sin(Math.PI/4 + Math.PI/2);
    double TurnVectorXM3 = turnX * Math.cos(Math.PI/4 + Math.PI + Math.PI/2); // back right 45+180
    double TurnVectorYM3 = turnX * Math.sin(Math.PI/4 + Math.PI + Math.PI/2);
    double TurnVectorXM4 = turnX * Math.cos(Math.PI/4 + Math.PI/2 + Math.PI/2 + Math.PI); // back left 45+90
    double TurnVectorYM4 = turnX * Math.sin(Math.PI/4 + Math.PI/2 + Math.PI/2 + Math.PI);


    // Add the vectors
    double M1VectorAddedX = TurnVectorXM1 + speedX;
    double M1VectorAddedY = TurnVectorYM1 + speedY;
    double M2VectorAddedX = TurnVectorXM2 + speedX;
    double M2VectorAddedY = TurnVectorYM2 + speedY;
    double M3VectorAddedX = TurnVectorXM3 + speedX;
    double M3VectorAddedY = TurnVectorYM3 + speedY;
    double M4VectorAddedX = TurnVectorXM4 + speedX;
    double M4VectorAddedY = TurnVectorYM4 + speedY;

    // Convert to angle
    double M1VectorAngle = Math.atan2(M1VectorAddedX, M1VectorAddedY) * 180/Math.PI;
    double M2VectorAngle = Math.atan2(M2VectorAddedX, M2VectorAddedY) * 180/Math.PI;
    double M3VectorAngle = Math.atan2(M3VectorAddedX, M3VectorAddedY) * 180/Math.PI;
    double M4VectorAngle = Math.atan2(M4VectorAddedX, M4VectorAddedY) * 180/Math.PI;
    
    // Get the vector length
    double M1VectorLength = Math.sqrt(Math.pow(M1VectorAddedX, 2) + Math.pow(M1VectorAddedY, 2));
    double M2VectorLength = Math.sqrt(Math.pow(M2VectorAddedX, 2) + Math.pow(M2VectorAddedY, 2));
    double M3VectorLength = Math.sqrt(Math.pow(M3VectorAddedX, 2) + Math.pow(M3VectorAddedY, 2));
    double M4VectorLength = Math.sqrt(Math.pow(M4VectorAddedX, 2) + Math.pow(M4VectorAddedY, 2));

    // Get the max vector length
    double vectorLengthMax1 = Math.max(Math.abs(M1VectorLength), Math.abs(M2VectorLength));
    double vectorLengthMax2 = Math.max(Math.abs(M3VectorLength), Math.abs(M4VectorLength));
    double vectorLengthMaxT = Math.max(vectorLengthMax1, vectorLengthMax2);
    
    SmartDashboard.putNumber("Vector Max Length", vectorLengthMaxT);
    
    // Get the normalized vectors if the max is greater than 1
    double M1VectorLengthNorm = 0;
    double M2VectorLengthNorm = 0;
    double M3VectorLengthNorm = 0;
    double M4VectorLengthNorm = 0;

    if (vectorLengthMaxT >= 1) {
      M1VectorLengthNorm = M1VectorLength / vectorLengthMaxT;
      M2VectorLengthNorm = M2VectorLength / vectorLengthMaxT;
      M3VectorLengthNorm = M3VectorLength / vectorLengthMaxT;
      M4VectorLengthNorm = M4VectorLength / vectorLengthMaxT;
    } else {
      M1VectorLengthNorm = M1VectorLength;
      M2VectorLengthNorm = M2VectorLength;
      M3VectorLengthNorm = M3VectorLength;
      M4VectorLengthNorm = M4VectorLength;
    } 

    double maxSpeed = 1;

    // set the module angles and speeds
    frontRightMod.setAngle(M1VectorAngle);
    frontLeftMod.setAngle(M2VectorAngle);
    backRightMod.setAngle(M3VectorAngle);
    backLeftMod.setAngle(M4VectorAngle);

    frontRightMod.setSpeed(accelLimiter.calculate(MathUtil.clamp(M1VectorLengthNorm, -maxSpeed, maxSpeed)));
    frontLeftMod.setSpeed(accelLimiter.calculate(MathUtil.clamp(M2VectorLengthNorm, -maxSpeed, maxSpeed)));
    backRightMod.setSpeed(accelLimiter.calculate(MathUtil.clamp(M3VectorLengthNorm, -maxSpeed, maxSpeed)));
    backLeftMod.setSpeed(accelLimiter.calculate(MathUtil.clamp(M4VectorLengthNorm, -maxSpeed, maxSpeed)));

    //TODO complete and use This:
    // Create a list of the turn vectors
    // double[] TurnVectorsX = {};
    // double[] TurnVectorsY = {};

    // for(int i = 0; i<4; i++){
    //   TurnVectorsX[i] = turnX * Math.cos(Math.PI/4 + ((Math.PI/2)*i));
    //   TurnVectorsX[i] = turnX * Math.sin(Math.PI/4 + ((Math.PI/2)*i));
    // }

    // SmartDashboard.putNumberArray("TurnVectorsX", TurnVectorsX);
    // SmartDashboard.putNumberArray("TurnVectorsY", TurnVectorsY);

    // double[] VectorsAddedX = {};
    // double[] VectorsAddedY = {};

    // for(int i = 0; i<4; i++){
    //   VectorsAddedX[i] = TurnVectorsX[i] + speedX;
    //   VectorsAddedY[i] = TurnVectorsY[i] + speedY;
    // }

    // SmartDashboard.putNumberArray("AddedVectorsX", VectorsAddedX);
    // SmartDashboard.putNumberArray("AddedVectorsY", VectorsAddedY);
  }
}
