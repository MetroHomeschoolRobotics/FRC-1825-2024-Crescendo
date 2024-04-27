// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAtAmp;
import frc.robot.commands.AimAtSpeakerAdjustable;
import frc.robot.commands.GoToSpeaker;
import frc.robot.commands.LobShot;
import frc.robot.commands.ReverseShooter;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunWrist;
import frc.robot.commands.ShootToSpeaker;
import frc.robot.commands.SetRobotPoseToSpeaker;
import frc.robot.commands.swervedrive.auto.AutoIntake;
import frc.robot.commands.swervedrive.auto.IntakeBackwards;
import frc.robot.commands.swervedrive.auto.LowerElevator;
import frc.robot.commands.swervedrive.auto.SetWristToAngle;
import frc.robot.commands.swervedrive.auto.ShootToAngle;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.logging.FieldView;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  private final Intake intake = new Intake();

  private final Shooter shooter;

  private final Elevator elevator = new Elevator();

  private final Wrist wrist = new Wrist();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  private final CommandXboxController m_manipulatorController = new CommandXboxController(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  private final RunElevator runElevator = new RunElevator(elevator, wrist, m_manipulatorController);
  private final RunWrist runWrist = new RunWrist(wrist, intake, elevator, m_manipulatorController);

  public SendableChooser<Command> _autoChooser = new SendableChooser<>();
  public SendableChooser<Command> _driveController = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    shooter = new Shooter(drivebase);

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    getAutoChooserOptions();
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    FieldView.publish();
    SmartDashboard.putNumber("SetWriteAngle", 0);
    SmartDashboard.putNumber("SetShooterSpeed", 2000);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.povUp().whileTrue(new SetRobotPoseToSpeaker(drivebase, driverXbox));
    driverXbox.rightTrigger().whileTrue(new GoToSpeaker(drivebase, shooter));
    
    m_manipulatorController.leftBumper().whileTrue(new RunIntake(intake, false, shooter, wrist).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    m_manipulatorController.rightBumper().whileTrue(new RunIntake(intake, true, shooter, wrist).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));  
    m_manipulatorController.x().whileTrue(new RunShooter(shooter, wrist));
    m_manipulatorController.a().whileTrue(new AimAtAmp(wrist, shooter, elevator).andThen(new SetWristToAngle(wrist, 60).alongWith(new LowerElevator(elevator))));// TODO test this
    // m_manipulatorController.y().whileTrue(new GoToSpeaker(drivebase, shooter));
    // m_manipulatorController.y().whileTrue(new ShootToSpeaker(shooter, wrist));
    m_manipulatorController.y().whileTrue(new ShootToSpeaker(shooter, wrist, drivebase));

    m_manipulatorController.povLeft().whileTrue(new ReverseShooter(shooter));
    m_manipulatorController.povUp().whileTrue(new ShootToAngle(shooter, wrist, 30));
    m_manipulatorController.povRight().whileTrue(new ShootToAngle(shooter, wrist, 23.5));// 23.8
    m_manipulatorController.povDown().whileTrue(new LobShot(shooter, wrist, 33));

    driverXbox.povDown().whileTrue(new GoToSpeaker(drivebase, shooter));

    
    
    CommandScheduler.getInstance().setDefaultCommand(elevator, runElevator);
    CommandScheduler.getInstance().setDefaultCommand(wrist, runWrist);
    
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void setDriveMode()
  {
    drivebase.setDefaultCommand(_driveController.getSelected());
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void getAutoChooserOptions() {
    NamedCommands.registerCommand("ShootAtBase", new RunShooter(shooter, wrist)); //new WaitCommand(0.8));
    NamedCommands.registerCommand("IntakeNote", new IntakeBackwards(intake, drivebase, wrist, shooter));
    NamedCommands.registerCommand("IntakeNote2", new AutoIntake(intake, shooter, wrist));
    NamedCommands.registerCommand("ShootToSpeaker", new ShootToSpeaker(shooter, wrist, drivebase).andThen(new SetWristToAngle(wrist, 60)));
    NamedCommands.registerCommand("ShootToAngle1", new ShootToAngle(shooter, wrist, 16));
    NamedCommands.registerCommand("ShootToAngle2", new ShootToAngle(shooter, wrist, 21)); // new WaitCommand(0.8));
    NamedCommands.registerCommand("ShootToAngle4", new ShootToAngle(shooter, wrist, 25));
    NamedCommands.registerCommand("TurnToSpeaker", new GoToSpeaker(drivebase, shooter));

    _autoChooser.setDefaultOption("No Auto", new WaitCommand(10));;;;;;;;;;;


    _autoChooser.addOption("2 NoteAuto (3 (3) )", drivebase.getAutonomousCommand("PickUpNote 3 (3)"));
    _autoChooser.addOption("3 NoteAuto (2,4 (2))", drivebase.getAutonomousCommand("PickUpNote 2, 4"));
    _autoChooser.addOption("3.5 NoteAuto (6,7,8 (3))", drivebase.getAutonomousCommand("PickUpNote 6, 7, 8 (3)"));
    _autoChooser.addOption("4 Note Auto (1,2,4 (2))", drivebase.getAutonomousCommand("PickUpNote 2, 1, 4"));
    _autoChooser.addOption("4 Note Auto (1,4,5 (1))", drivebase.getAutonomousCommand("PickUpNote 1, 4, 5 (1)"));
    _autoChooser.addOption("4 Note Auto (3,7,8 (3))", drivebase.getAutonomousCommand("PickUpNote 3, 7, 8 (3)"));
    _autoChooser.addOption("4 Note Auto (2,3,4 (1))", drivebase.getAutonomousCommand("PickUpNote 2, 3, 4 (2)"));
    _autoChooser.addOption("5 Note Auto (1,2,3,4 (2))", drivebase.getAutonomousCommand("PickUpNote 1, 2, 3, 4 (2)"));
    _autoChooser.addOption("5 Note Auto (1,2,3,4 (3)) (Untested)", drivebase.getAutonomousCommand("PickUpNote 1, 2, 3, 4 (3)"));
    _autoChooser.addOption("5 Note Auto (1,2,3,8 (1)) (Untested)", drivebase.getAutonomousCommand("PickUpNote 1, 2, 3, 8 (1)"));
    _autoChooser.addOption("Just Shoot", new RunShooter(shooter, wrist));

    _driveController.addOption("FieldOrientedDirectDrive", drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),

        () -> driverXbox.getRightY()));
    _driveController.addOption(("FieldOrientedAnglularVelocity"), drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX()));


    SmartDashboard.putData(_autoChooser);
    SmartDashboard.putData(_driveController);
  }

  public Command getAutonomousCommand() {
    return _autoChooser.getSelected();
  }
}
