package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveModule extends SubsystemBase {
  
  // encoders and motors
  private CANcoder angleEncoder;
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;
  
  // turn pid USED FOR BRUTE FORCED
  private PIDController turnPID = new PIDController(0.005, 0, 0.0001);

  // speed controller
  private double ku = 0.1; // contant oscillation kp
  private double tu = 4/11; // time contant for oscillation

  private PIDController speedController;
  private SimpleMotorFeedforward feedforwardSpeedController;

  private ProfiledPIDController turnSpeedController;
  private SimpleMotorFeedforward feedforwardTurnController;
  
  // offset of the angle encoders & placement on the robot
  private String placement;
  private double angleOffset;

  // stuff for dist per rotation
  private double wheelRadiusInches = 2;
  private double gearRatio = 6.12;
  
  public SwerveModule(
    String modulePlacement, 
    int angleEncoderID, int angleMotorID, 
    int driveMotorID,
    boolean angleMotorReversed, boolean driveMotorReversed,
    double _angleOffset,
    double TKs, double TKv,
    double TKu, double TTu) {

    // double DKs, double DKv,
    // double DKu, double DTu
    
    // Set up the motors and encoders
    angleEncoder = new CANcoder(angleEncoderID);
    angleMotor = new CANSparkMax(angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    angleMotor.setInverted(angleMotorReversed);

    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setIdleMode(IdleMode.kBrake);


    driveMotor.setSmartCurrentLimit(20, 60);
    angleMotor.setSmartCurrentLimit(20, 60);

    driveMotor.getEncoder().setPositionConversionFactor((Units.inchesToMeters(wheelRadiusInches*2*Math.PI)*2/(gearRatio))); 
    driveMotor.getEncoder().setVelocityConversionFactor((Units.inchesToMeters(wheelRadiusInches*2*Math.PI)/(gearRatio))/60);
    
    // Set up the PID and Feedforward
    speedController = new PIDController(0.01, 0, 0);
    feedforwardSpeedController = new SimpleMotorFeedforward(0, 2.35, 0.39); //TKu*0.2, 0.4*TKu/TTu, 0.06666666*TKu*TTu
// TKu, 0, 0
    turnSpeedController = new ProfiledPIDController(TKu, 0, 0, Constants.autoConstants.spinPIDConstraints);
    feedforwardTurnController = new SimpleMotorFeedforward(TKs, TKv);

    // FOR BRUTE FORCED
    turnPID.enableContinuousInput(-180, 180);

    turnSpeedController.enableContinuousInput(-180,180);
    placement = modulePlacement;

    angleOffset = _angleOffset;
  }
  
  public void periodic() {

    
    SmartDashboard.putNumber(placement + " Module Angle", Math.round(getModuleAngle()) );
    SmartDashboard.putNumber(placement + " Velocity", getVelocity());
  }
  

  /*
   * Angle Motor
   */
  public double getModuleAngle() {
    return (angleEncoder.getAbsolutePosition().getValueAsDouble()*(360/1))+angleOffset; // in degrees
  }
  public Rotation2d wheelRotation2d() {
    return new Rotation2d(getModuleAngle()*(Math.PI/180));
  }
  public void rotateModuleVolts(){
    angleMotor.setVoltage(0.39);
  }

  /* for brute forced equasion */
  public void setAngle(double theta) {

    // reverses the wheels instead of spinning 180°
    double actualAngleDist = getModuleAngle()-theta;

    if(actualAngleDist >= 90 || actualAngleDist <=-90){
      theta = theta+180;
      driveMotor.setInverted(true);
    }else{
      driveMotor.setInverted(false);
    }

    double anglePID = MathUtil.clamp(turnPID.calculate(getModuleAngle(), theta), -1, 1);
    
    angleMotor.set(anglePID);
  }
  
  public double getAngleDifference() {
    return turnPID.getPositionError();
  }

  public Boolean angleTurnFinished() {
    return turnPID.atSetpoint();
  }

  
  
  /*
   * Drive Motor
   */

  public double getDistance() {
    return driveMotor.getEncoder().getPosition();
  }
  public double getVelocity() {
    return driveMotor.getEncoder().getVelocity();
  }

  public void resetDistance() {
    driveMotor.getEncoder().setPosition(0);
  }

  public void setSpeed(double speed) {
    // this slowes the drive motor by how far the angle is from it's setpoint
    //speed = Math.abs(speed * Math.cos(getAngleDifference()));

    SmartDashboard.putNumber(placement + " Speed", speed);

    driveMotor.set(speed);
  }

  /*
   * Both Motors
   */

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), wheelRotation2d());
  }
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDistance(), wheelRotation2d());
  }
  public Translation2d getTranslation() {
    return new Translation2d(getDistance(), wheelRotation2d());
  }
  

  public void stopMotors() {
    driveMotor.set(0);
    angleMotor.set(0);
  }

  // used for destinguishing between modules
  public String getPlacement() {
    return placement;
  }
  
  // Drive the Motors
  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getModuleState().angle);

    state.speedMetersPerSecond *= state.angle.minus(wheelRotation2d()).getCos();
    
    SmartDashboard.putNumber(placement + " angleInput", state.angle.getDegrees());
    SmartDashboard.putNumber(placement + " Velocity Setpoint", state.speedMetersPerSecond);

    double driveOutput = -speedController.calculate(getVelocity(), state.speedMetersPerSecond);
    double driveFeedForward = feedforwardSpeedController.calculate(state.speedMetersPerSecond);

    double turnOutput = turnSpeedController.calculate(getModuleAngle(), state.angle.getDegrees());
    double turnFeedForward = feedforwardTurnController.calculate(turnSpeedController.getSetpoint().velocity);
    
    driveMotor.setVoltage(driveOutput + driveFeedForward);
    angleMotor.setVoltage(turnOutput + turnFeedForward);// 
  }
}
