package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.swerveConstants;
import frc.robot.lib.field.FieldInfo;
import frc.robot.subsystems.Swerve.SwerveEstimator;
import frc.robot.subsystems.Swerve.SwerveKinematics;

public class Drivetrain extends SubsystemBase {

  private SwerveModule[] modules;
  private SwerveKinematics kinematics;
  private SwerveEstimator estimator;

  private SwerveModulePosition[] prevPositions;
  private Rotation2d prevGyroAngle;

  // this gets each swerve module so I won't have to get each motor and encoder
  // individually
  private SwerveModule frontRightMod = new SwerveModule(
      "Front Right",
      swerveConstants.swerveModuleFR.angleEncoderID, swerveConstants.swerveModuleFR.angleMotorID,
      swerveConstants.swerveModuleFR.driveMotorID,
      swerveConstants.swerveModuleFR.angleMotorReversed, swerveConstants.swerveModuleFR.driveMotorReversed,
      swerveConstants.swerveModuleFR.angleOffset,
      0.38, 0.009,
      0.1483, 6);

  private SwerveModule frontLeftMod = new SwerveModule(
      "Front Left",
      swerveConstants.swerveModuleFL.angleEncoderID, swerveConstants.swerveModuleFL.angleMotorID,
      swerveConstants.swerveModuleFL.driveMotorID,
      swerveConstants.swerveModuleFL.angleMotorReversed, swerveConstants.swerveModuleFL.driveMotorReversed,
      swerveConstants.swerveModuleFL.angleOffset,
      0.38, 0.009,
      0.1483, 6);

  private SwerveModule backRightMod = new SwerveModule(
      "Back Right",
      swerveConstants.swerveModuleBR.angleEncoderID, swerveConstants.swerveModuleBR.angleMotorID,
      swerveConstants.swerveModuleBR.driveMotorID,
      swerveConstants.swerveModuleBR.angleMotorReversed, swerveConstants.swerveModuleBR.driveMotorReversed,
      swerveConstants.swerveModuleBR.angleOffset,
      0.38, 0.009,
      0.1483, 6);

  private SwerveModule backLeftMod = new SwerveModule(
      "Back Left",
      swerveConstants.swerveModuleBL.angleEncoderID, swerveConstants.swerveModuleBL.angleMotorID,
      swerveConstants.swerveModuleBL.driveMotorID,
      swerveConstants.swerveModuleBL.angleMotorReversed, swerveConstants.swerveModuleBL.driveMotorReversed,
      swerveConstants.swerveModuleBL.angleOffset,
      0.38, 0.009,
      0.1483, 6);

  private AHRS gyro = new AHRS();

  private Field2d field = new Field2d();

  // TODO delete or use this: private SlewRateLimiter accelLimiter = new
  // SlewRateLimiter(.99);
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.autoConstants.swerveKinematics,
      gyro.getRotation2d(), getModulePositions());

  public Drivetrain() {
    resetDistance();
    
    this.kinematics = new SwerveKinematics(geTranslation2ds(),Constants.autoConstants.maxSpeedMetersPerSecond);
    this.estimator = new SwerveEstimator(FieldInfo.CRESCENDO_2024);
    // Auto Builder MUST BE AT BOTTOM
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig(
            new PIDConstants(4, 0.0, 0.0),
            new PIDConstants(2, 0.0, 0.0),
            Constants.autoConstants.maxSpeedMetersPerSecond,
            0.367, // in meters
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();

          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  public void periodic() {
    // update the odometry
    odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()), getModulePositions());

    SmartDashboard.putNumber("Gyro rotation", getRotation());
    SmartDashboard.putNumber("RadiansPerSecond", gyro.getRate() * (Math.PI / 180));
    SmartDashboard.putNumber("Average Distance", getAvgDist());
    SmartDashboard.putNumber("Assumed Distance", odometry.getPoseMeters().getX());
    SmartDashboard.putData("Field", field);

    SmartDashboard.putData(gyro);

    field.setRobotPose(odometry.getPoseMeters());

    // Update estimator
    // Do refresh here, so we get the most up-to-date data
    SwerveModulePosition[] positions = getModulePositions();
    Rotation2d gyroAngle = gyro.getRotation2d();
    if (prevPositions != null) {
      Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);
     // Logger.recordOutput("Drive/Estimated Twist", twist);

      // We trust the gyro more than the kinematics estimate
      if (RobotBase.isReal() && gyro.isConnected()) {
        twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();
      }

      estimator.update(twist);
    }
    prevPositions = positions;
    prevGyroAngle = gyroAngle;

    SmartDashboard.putData("Field", field);
  }

  /*
   * Gyro
   */
  public void resetGyro() {
    gyro.reset();
  }

  public double getRotation() {
    return gyro.getAngle();
  }

  /*
   * Read from Drive Encoders
   */

  // Generally, this will be more accurate for straight lines...
  public double getAvgDist() {
    double avgDist = (frontLeftMod.getDistance() + frontRightMod.getDistance() + backLeftMod.getDistance()
        + backRightMod.getDistance()) / 4;
    return avgDist;
  }

  public void resetDistance() {
    frontRightMod.resetDistance();
    frontLeftMod.resetDistance();
    backRightMod.resetDistance();
    backLeftMod.resetDistance();
  }

  /*
   * Auto Stuffs
   */

  // reset the odometry to the current pose on the field
  public void resetOdometry(Pose2d position) {
    odometry.resetPosition(new Rotation2d(getRotation()), getModulePositions(), position);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.autoConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    // swerveKinematics().toChassisSpeeds(getModuleStates()); TODO test this
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = { frontLeftMod.getModuleState(), frontRightMod.getModuleState(),
        backLeftMod.getModuleState(), backRightMod.getModuleState() };
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = { frontLeftMod.getModulePosition(), frontRightMod.getModulePosition(),
        backLeftMod.getModulePosition(), backRightMod.getModulePosition() };
    return modulePositions;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveDriveKinematics swerveKinematics() {
    return new SwerveDriveKinematics(frontLeftMod.getTranslation(), frontRightMod.getTranslation(),
        backLeftMod.getTranslation(), backRightMod.getTranslation());
  }

  public Translation2d[] geTranslation2ds() {
    Translation2d[] translation2ds = {frontLeftMod.getTranslation(), frontRightMod.getTranslation(),
        backLeftMod.getTranslation(), backRightMod.getTranslation()};
    return translation2ds;
  }
  // Generates a path command
  public Command followPathCommand(String pathName, boolean resetOdometry) {

    resetGyro();

    // TODO test which works ☺☺☻☻
    return new PathPlannerAuto(pathName);
    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // return AutoBuilder.followPath(path);

    // if(resetOdometry == true) {
    // resetGyro(); // dont know if you need this TODO try without
    // resetDistance();
    // resetOdometry(path.getStartingDifferentialPose());// sets the odometry to the
    // first position on the map (in meters)
    // }
    // //path.getStartingDifferentialPose().getRotation().getDegrees();

    // return new FollowPathHolonomic(
    // path,
    // this::getPose,
    // this::getRobotRelativeSpeeds,
    // this::driveRobotRelative,
    // new HolonomicPathFollowerConfig(
    // new PIDConstants(0.5, 0.0, 0.0),
    // new PIDConstants(0.1, 0.0, 0.0),
    // Constants.autoConstants.maxSpeedMetersPerSecond,
    // 0.367, // in meters
    // new ReplanningConfig()),
    // () -> {
    // var alliance = DriverStation.getAlliance();

    // if (alliance.isPresent()) {
    // return alliance.get() == DriverStation.Alliance.Red;
    // }
    // return false;
    // },
    // this);
  }

  /*
   * Drive the robot
   */

  /* for autos */
  // this does the calculations for swerve auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.autoConstants.maxSpeedMetersPerSecond);

    frontLeftMod.setDesiredState(desiredStates[0]);
    frontRightMod.setDesiredState(desiredStates[1]);
    backLeftMod.setDesiredState(desiredStates[2]);
    backRightMod.setDesiredState(desiredStates[3]);
  }

  // this actually drives the robot
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = swerveKinematics().toSwerveModuleStates(speeds);

    setModuleStates(swerveModuleStates);
  }
  public void spinModuleVolts() {
    frontRightMod.rotateModuleVolts();
    frontLeftMod.rotateModuleVolts();
    backRightMod.rotateModuleVolts();
    backLeftMod.rotateModuleVolts();
  }

  /* for controllers */
  //
  public void driveModules(double translateX, double translateY, double rotationX, Boolean fieldOriented,
      double periodSeconds) {

    translateX = Constants.autoConstants.maxSpeedMetersPerSecond * translateX;
    translateY = Constants.autoConstants.maxSpeedMetersPerSecond * translateY;
    rotationX = Constants.autoConstants.maxSpeedRadiansPerSecond * rotationX;

    SwerveModuleState[] swerveModuleStates = Constants.autoConstants.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translateY, translateX, -rotationX, gyro.getRotation2d())
                : new ChassisSpeeds(translateY, translateX, rotationX),
            1));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.autoConstants.maxSpeedMetersPerSecond);

    frontLeftMod.setDesiredState(swerveModuleStates[1]);
    frontRightMod.setDesiredState(swerveModuleStates[0]);
    backLeftMod.setDesiredState(swerveModuleStates[3]);
    backRightMod.setDesiredState(swerveModuleStates[2]);
  }

  // manually chugged equasion
  public void translateSpin(double speedX, double speedY, double turnX, Boolean boost) {
    turnX = -turnX;

    double maxSpeed = 0.8;

    // Get the translate angle and add the gyro angle to it
    double translateAngle = Math.atan2(speedY, speedX) * (180 / Math.PI) - gyro.getAngle();
    double translateSpeed = Math.sqrt(Math.pow(speedX, 2) + Math.pow(speedY, 2));

    SmartDashboard.putNumber("Translate Angle", translateAngle);

    speedX = translateSpeed * Math.cos(translateAngle * (Math.PI / 180));
    speedY = translateSpeed * Math.sin(translateAngle * (Math.PI / 180));

    // This converts everything to vectors and adds them ☺
    double TurnVectorXM1 = turnX * Math.cos(Math.PI / 4 + (Math.PI * 3) / 2 + Math.PI / 2 + Math.PI); // front right 45
                                                                                                      // + 270
    double TurnVectorYM1 = turnX * Math.sin(Math.PI / 4 + (Math.PI * 3) / 2 + Math.PI / 2 + Math.PI);
    double TurnVectorXM2 = turnX * Math.cos(Math.PI / 4 + Math.PI / 2); // front left 45
    double TurnVectorYM2 = turnX * Math.sin(Math.PI / 4 + Math.PI / 2);
    double TurnVectorXM3 = turnX * Math.cos(Math.PI / 4 + Math.PI + Math.PI / 2); // back right 45+180
    double TurnVectorYM3 = turnX * Math.sin(Math.PI / 4 + Math.PI + Math.PI / 2);
    double TurnVectorXM4 = turnX * Math.cos(Math.PI / 4 + Math.PI / 2 + Math.PI / 2 + Math.PI); // back left 45+90
    double TurnVectorYM4 = turnX * Math.sin(Math.PI / 4 + Math.PI / 2 + Math.PI / 2 + Math.PI);

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
    double M1VectorAngle = Math.atan2(M1VectorAddedX, M1VectorAddedY) * 180 / Math.PI;
    double M2VectorAngle = Math.atan2(M2VectorAddedX, M2VectorAddedY) * 180 / Math.PI;
    double M3VectorAngle = Math.atan2(M3VectorAddedX, M3VectorAddedY) * 180 / Math.PI;
    double M4VectorAngle = Math.atan2(M4VectorAddedX, M4VectorAddedY) * 180 / Math.PI;

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

    if (boost == true) {
      maxSpeed = 1;
    } else {
      maxSpeed = 0.7;
    }

    // set the module angles and speeds
    frontRightMod.setAngle(M1VectorAngle);
    frontLeftMod.setAngle(M2VectorAngle);
    backRightMod.setAngle(M3VectorAngle);
    backLeftMod.setAngle(M4VectorAngle);

    frontRightMod.setSpeed(M1VectorLengthNorm * maxSpeed);
    frontLeftMod.setSpeed(M2VectorLengthNorm * maxSpeed);
    backRightMod.setSpeed(M3VectorLengthNorm * maxSpeed);
    backLeftMod.setSpeed(M4VectorLengthNorm * maxSpeed);

    // frontRightMod.setSpeed(accelLimiter.calculate(M1VectorLengthNorm *
    // maxSpeed));
    // frontLeftMod.setSpeed(accelLimiter.calculate(M2VectorLengthNorm*maxSpeed));
    // backRightMod.setSpeed(accelLimiter.calculate(M3VectorLengthNorm*maxSpeed));
    // backLeftMod.setSpeed(accelLimiter.calculate(M4VectorLengthNorm*maxSpeed));

    // TODO complete and use This:
    // Create a list of the turn vectors
    // double[] TurnVectorsX = {};
    // double[] TurnVectorsY = {};

    // for(int i = 0; i<4; i++){
    // TurnVectorsX[i] = turnX * Math.cos(Math.PI/4 + ((Math.PI/2)*i));
    // TurnVectorsX[i] = turnX * Math.sin(Math.PI/4 + ((Math.PI/2)*i));
    // }

    // SmartDashboard.putNumberArray("TurnVectorsX", TurnVectorsX);
    // SmartDashboard.putNumberArray("TurnVectorsY", TurnVectorsY);

    // double[] VectorsAddedX = {};
    // double[] VectorsAddedY = {};

    // for(int i = 0; i<4; i++){
    // VectorsAddedX[i] = TurnVectorsX[i] + speedX;
    // VectorsAddedY[i] = TurnVectorsY[i] + speedY;
    // }

    // SmartDashboard.putNumberArray("AddedVectorsX", VectorsAddedX);
    // SmartDashboard.putNumberArray("AddedVectorsY", VectorsAddedY);
  }
}
