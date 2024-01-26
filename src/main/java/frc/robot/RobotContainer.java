// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Drivetrain r_drivetrain = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final Teleop r_teleop = new Teleop(m_driverController, r_drivetrain, new Robot().getPeriod());
      
  public SendableChooser<Command> _autoChooser = new SendableChooser<>();


      public void setDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(r_drivetrain, r_teleop);
      }
      
      public void init() {
        setDefaultCommands();
      }
      
      public RobotContainer() {
        init();
        configureBindings();
        getAutoChooserOptions();
      }

      
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(new ResetGyro(r_drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // TODO delete this
  public Command loadPathPlannerToHolonomicCommand(String filename) {
        filename = "pathplanner/generatedJSON/" + filename + ".wpilib.json";

        Trajectory trajectory;

        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException exception) {
          DriverStation.reportError("Unable to open trajectory: "+ filename, exception.getStackTrace());
          System.out.println("Unable to read trajectory: " + filename);
          return new InstantCommand();
        }

        ProfiledPIDController thetaController = new ProfiledPIDController(Constants.autoConstants.kpTurnVelocity, 0, 0, Constants.autoConstants.spinPIDConstraints);

        thetaController.enableContinuousInput(-180, 180);


        SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
          trajectory,
          r_drivetrain::getPose,
          Constants.autoConstants.swerveKinematics,
          new PIDController(Constants.autoConstants.kpDriveVelocity, 0, 0),
          new PIDController(Constants.autoConstants.kpDriveVelocity, 0, 0),
          thetaController,
          r_drivetrain::setModuleStates,
          r_drivetrain);
      
        SendableRegistry.setName(swerveCommand, "Swerve Command");

        return swerveCommand;
      }

  public void getAutoChooserOptions() {
    // TODO delete this
    _autoChooser.setDefaultOption("No Auto", new WaitCommand(10));

    _autoChooser.addOption("Test Auto", r_drivetrain.followPathCommand("Straight6Meters"));


    SmartDashboard.putData(_autoChooser);
  }

  public Command getAutonomousCommand() {
    return _autoChooser.getSelected();
  }
}
