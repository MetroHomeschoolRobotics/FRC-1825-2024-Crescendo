package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANcoder angleEncoder;

  private CANSparkMax angleMotor;
  
  private CANSparkMax driveMotor;
  
  private PIDController turnPID = new PIDController(0.005, 0, 0.0001);
  
  private String placement;

  private double angleOffset;
  
  public SwerveModule(
    String modulePlacement, 
    int angleEncoderID, int angleMotorID, 
    int driveMotorID,
    boolean angleMotorReversed, boolean driveMotorReversed,
    double _angleOffset) {
    angleEncoder = new CANcoder(angleEncoderID);

    angleMotor = new CANSparkMax(angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    angleMotor.setInverted(angleMotorReversed);

    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setIdleMode(IdleMode.kBrake);

    turnPID.enableContinuousInput(-180, 180);

    placement = modulePlacement;

    angleOffset = _angleOffset;
  }
  
  public void periodic() {
    SmartDashboard.putNumber(placement + " Module Angle", moduleAngle());
  }
  
  // Angle Motor
  public double moduleAngle() {
    return (angleEncoder.getAbsolutePosition().getValueAsDouble()*(360/1))+angleOffset;
  }
  
  public void setAngle(double theta) {
    
    // post angles to the smart dashboard 0-360
    //if(theta >= 0){
    SmartDashboard.putNumber(placement + " Angle", theta);
    //}else{
      //SmartDashboard.putNumber(placement + " Angle", theta+360);
    //}

    // reverses the wheels instead of spinning 180°
    double actualAngleDist = moduleAngle()-theta;

    if(actualAngleDist >= 90 || actualAngleDist <=-90){
      theta = theta+180;
      driveMotor.setInverted(true);
    }else{
      driveMotor.setInverted(false);
    }

    double anglePID = MathUtil.clamp(turnPID.calculate(moduleAngle(), theta), -1, 1);

    SmartDashboard.putNumber(placement + " PID", anglePID);
    
    angleMotor.set(anglePID);
  }
  
  public Boolean angleTurnFinished() {
    return turnPID.atSetpoint();
  }
  

  // Drive Motor
  public double getDistance() {
    return driveMotor.getEncoder().getPosition();
  }
  
  public void setSpeed(double speed) {
    SmartDashboard.putNumber(placement + " Speed", speed);
    driveMotor.set(speed);
  }
  
  // this could be used for destinguishing between modules
  public String getPlacement() {
    return placement;
  }
}
