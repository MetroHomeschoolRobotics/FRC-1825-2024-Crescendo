// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
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
import frc.robot.commands.DischargeShooter;
import frc.robot.commands.GoToSpeaker;
import frc.robot.commands.LobShot;
import frc.robot.commands.PrechargeShooter;
import frc.robot.commands.ReverseShooter;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunWrist;
import frc.robot.commands.ShootToSpeaker;
import frc.robot.commands.auto.AutoIntake;
import frc.robot.commands.auto.IntakeBackwards;
import frc.robot.commands.auto.LowerElevator;
import frc.robot.commands.auto.SetWristToAngle;
import frc.robot.commands.auto.ShootToAngle;
import frc.robot.commands.SetRobotPoseToSpeaker;
import frc.robot.logging.FieldView;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();
  private final Wrist wrist = new Wrist();
  // private final PhotonCamera camera = new PhotonCamera("OV5647");
  private final Shooter shooter = new Shooter(drivebase);

  // Define the controllers
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_manipulatorController = new CommandXboxController(1);
  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final RunElevator runElevator = new RunElevator(elevator, wrist, m_manipulatorController);
  private final RunWrist runWrist = new RunWrist(wrist, intake, elevator, m_manipulatorController);

  // Define the dropdown menus we will put on the dashboard
  public SendableChooser<Command> _autoChooser = new SendableChooser<>();
  public SendableChooser<Command> _driveController = new SendableChooser<>();
  public double turnDeadband = 0.2;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRightX(), turnDeadband));
    // Added negative to the controllers to make the directions work                                                              

    getAutoChooserOptions();
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); 
    // Made anglularVelocity the default since everyone prefers that. J.B.

    FieldView.publish();
    // SmartDashboard.putNumber("SetWriteAngle", 0);
    // SmartDashboard.putNumber("SetShooterSpeed", 2000);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Bind all your commands to controller conditions
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.povUp().whileTrue(new SetRobotPoseToSpeaker(drivebase, driverXbox));
    driverXbox.rightTrigger().whileTrue(new GoToSpeaker(drivebase, shooter));
    driverXbox.b().whileTrue(drivebase.driveToPose(new Pose2d(2.90, 5.54, null)));
    m_manipulatorController.leftBumper().whileTrue(new RunIntake(intake, false, shooter, wrist)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    m_manipulatorController.rightBumper().whileTrue(new RunIntake(intake, true, shooter, wrist)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    m_manipulatorController.x().whileTrue(new RunShooter(shooter, wrist));
    double wristSpeed = 0.8;
    m_manipulatorController.a()
        .whileTrue(new AimAtAmp(wrist, shooter, elevator)
            .andThen(new SetWristToAngle(wrist, 55, wristSpeed).alongWith(new LowerElevator(elevator))))
        .whileFalse(new SetWristToAngle(wrist, 58, wristSpeed));
    m_manipulatorController.b().whileTrue(new AimAtSpeakerAdjustable(wrist, shooter));
    // m_manipulatorController.y().whileTrue(new GoToSpeaker(drivebase, shooter));
    // m_manipulatorController.y().whileTrue(new ShootToSpeaker(shooter, wrist));
    //m_manipulatorController.y().whileTrue(new ShootToSpeaker(shooter, wrist, drivebase));
    m_manipulatorController.y().whileTrue(new PrechargeShooter(shooter, wrist)).whileFalse(new DischargeShooter(shooter, wrist));
    // m_manipulatorController.start().onTrue(shooter.incrementTrimCommand());  TODO: WHY IS THIS HERE!!!
    // m_manipulatorController.back().onTrue(shooter.decrementTrimCommand());

    m_manipulatorController.povLeft().whileTrue(new ReverseShooter(shooter));
    m_manipulatorController.povUp().whileTrue(new ShootToAngle(shooter, wrist, 30));
    m_manipulatorController.povRight().whileTrue(new ShootToAngle(shooter, wrist, 23.5));// at one point was 23.8
    m_manipulatorController.povDown().whileTrue(new LobShot(shooter, wrist, 33));

    //driverXbox.rightBumper().whileTrue(new RunAimAtTarget(camera, drivebase, intake, shooter)
    //    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // Commented this out because it crashes the code

    CommandScheduler.getInstance().setDefaultCommand(elevator, runElevator);
    CommandScheduler.getInstance().setDefaultCommand(wrist, runWrist);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void setDriveMode() {
    drivebase.setDefaultCommand(_driveController.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void getAutoChooserOptions() {
    // These are the named commands for the auto routines
    NamedCommands.registerCommand("ShootAtBase", new RunShooter(shooter, wrist)); // new WaitCommand(0.8));
    NamedCommands.registerCommand("IntakeNote", new IntakeBackwards(intake, drivebase, wrist, shooter));
    NamedCommands.registerCommand("IntakeNote2", new AutoIntake(intake, shooter, wrist));
    NamedCommands.registerCommand("ShootToSpeaker",
        new ShootToSpeaker(shooter, wrist, drivebase).andThen(new SetWristToAngle(wrist, 60)));
    NamedCommands.registerCommand("ShootToAngle1", new ShootToAngle(shooter, wrist, 16));
    NamedCommands.registerCommand("ShootToAngle2", new ShootToAngle(shooter, wrist, 21)); // new WaitCommand(0.8));
    NamedCommands.registerCommand("ShootToAngle4", new ShootToAngle(shooter, wrist, 25));
    NamedCommands.registerCommand("TurnToSpeaker", new GoToSpeaker(drivebase, shooter));
    NamedCommands.registerCommand("LobNote", new LobShot(shooter, wrist, 33.0));

    // Sets the default option on the auto chooser to No Auto
    _autoChooser.setDefaultOption("No Auto", new WaitCommand(10));

    // Defines all the auto chooser options on the dropdown and their associated
    // functions
    _autoChooser.addOption("Mobility (1)", drivebase.getAutonomousCommand("Mobility (1)"));
    _autoChooser.addOption("Yank 1 Note (1)", drivebase.getAutonomousCommand("Yank 1 Note (1)"));
    _autoChooser.addOption("Eat 2 Notes (2)", drivebase.getAutonomousCommand("Eat 2 Notes (2)"));
    _autoChooser.addOption("Steal Midline Notes (3)", drivebase.getAutonomousCommand("Steal Midline Notes (3)"));
    _autoChooser.addOption("3 NoteAuto (2,4 (2))", drivebase.getAutonomousCommand("PickUpNote 2, 4"));
    _autoChooser.addOption("3.5 NoteAuto (6,7,8 (3))", drivebase.getAutonomousCommand("PickUpNote 6, 7, 8 (3)"));
    _autoChooser.addOption("4 Note Auto (1,2,4 (2))", drivebase.getAutonomousCommand("PickUpNote 2, 1, 4"));
    _autoChooser.addOption("4 Note Auto (1,4,5 (1))", drivebase.getAutonomousCommand("PickUpNote 1, 4, 5 (1)"));
    _autoChooser.addOption("4 Note Auto (3,7,8 (3))", drivebase.getAutonomousCommand("PickUpNote 3, 7, 8 (3)"));
    _autoChooser.addOption("4 Note Auto (2,3,4 (1))", drivebase.getAutonomousCommand("PickUpNote 2, 3, 4 (2)"));
    _autoChooser.addOption("5 Note Auto (1,2,3,4 (2))", drivebase.getAutonomousCommand("PickUpNote 1, 2, 3, 4 (2)"));
    _autoChooser.addOption("5 Note Auto (1,2,3,4 (3)) (Untested)",
        drivebase.getAutonomousCommand("PickUpNote 1, 2, 3, 4 (3)"));
    _autoChooser.addOption("5 Note Auto (1,2,3,8 (1)) (Untested)",
        drivebase.getAutonomousCommand("PickUpNote 1, 2, 3, 8 (1)"));
    _autoChooser.addOption("Just Shoot", new RunShooter(shooter, wrist));
    _autoChooser.addOption("Straight6Meter", drivebase.getAutonomousCommand("Straight6Meters"));
    _autoChooser.addOption("Straight2Meters", drivebase.getAutonomousCommand("Straight2Meters"));
    _autoChooser.addOption("Forward 3 then Back 1", drivebase.getAutonomousCommand("F3 then B1"));

    // Defines the drive options in the dropdown on the dashboard
    _driveController.addOption("FieldOrientedDirectDrive", drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> driverXbox.getRightY()));
        // TODO Should check if this command needs negatives
  
    _driveController.addOption(("FieldOrientedAnglularVelocity"), drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
        () -> MathUtil.applyDeadband(-driverXbox.getRightX(), turnDeadband)));
        // Added negative to the controllers to make the directions work

    // Puts the two dropdown choosers on the dashboard
    SmartDashboard.putData(_autoChooser);
    SmartDashboard.putData(_driveController);
  }

  // Retrieves the auto choosen on the dashboard and returns it
  public Command getAutonomousCommand() {
    return _autoChooser.getSelected();
  }
}
