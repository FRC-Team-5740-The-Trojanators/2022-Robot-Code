// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.HIDConstants;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.HoldIntakeCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(false);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final CatapultSubsystem m_catapultSubsystem = new CatapultSubsystem();

  XboxController m_driverController = new XboxController(HIDConstants.k_DriverControllerPort);
  XboxController m_operatorController = new XboxController(HIDConstants.k_OperatorControllerPort);

  private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);
  private final AutonomousDrive m_autonomousDrive = new AutonomousDrive(m_driveSubsystem);
  private final IntakeRunCommand m_intakeRunCommand = new IntakeRunCommand(m_intakeSubsystem);
  private final IntakeReverseCommand m_intakeReverseCommand = new IntakeReverseCommand(m_intakeSubsystem);
  private final HoldIntakeCommand m_holdIntakeCommand = new HoldIntakeCommand(m_intakeSubsystem);
  private final CatapultCommand m_catapultCommand = new CatapultCommand(m_catapultSubsystem);

  public static JoystickButton intakeFlip, intakeRun, intakeReverse, climbAngle, holdIntake, launchCatapult;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(m_driveCommand);

    m_driveSubsystem.resetIMU();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    intakeFlip = new JoystickButton(m_driverController , HIDConstants.kA);
    intakeRun = new JoystickButton(m_driverController , HIDConstants.kLB);
    intakeReverse = new JoystickButton(m_driverController, HIDConstants.kStart);
  
    intakeFlip.toggleWhenPressed(new StartEndCommand(m_intakeSubsystem::retractIntake, m_intakeSubsystem::extendIntake, m_intakeSubsystem));
    intakeRun.whileHeld(m_intakeRunCommand);
    intakeReverse.whileHeld(m_intakeReverseCommand);

    climbAngle.toggleWhenPressed(new StartEndCommand(m_climbSubsystem::straightClimb, m_climbSubsystem::angleClimb, m_climbSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An ExampleCommand will run in autonomous
    return m_autonomousDrive;
  }
}
