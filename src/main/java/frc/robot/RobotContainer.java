// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.HIDConstants;
import frc.robot.commands.AngleClimbCommand;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.LoadCatapultCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.StraightenClimbCommand;
import frc.robot.commands.MoveServoCommand;
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
  private final RunIntakeCommand m_runIntakeCommand = new RunIntakeCommand(m_intakeSubsystem);
  private final LoadCatapultCommand m_loadCatapultCommand = new LoadCatapultCommand(m_intakeSubsystem);
  private final ReverseIntakeCommand m_reverseIntakeCommand = new ReverseIntakeCommand(m_intakeSubsystem);
  private final CatapultCommand m_catapultCommand = new CatapultCommand(m_catapultSubsystem, m_intakeSubsystem);
  private final ClimbUpCommand m_climbUpCommand = new ClimbUpCommand(m_climbSubsystem, m_intakeSubsystem);
  private final ClimbDownCommand m_climbDownCommand = new ClimbDownCommand(m_climbSubsystem, m_intakeSubsystem);
  private final MoveServoCommand m_moveServoCommand = new MoveServoCommand(m_climbSubsystem);
  private final RetractIntakeCommand m_retractIntakeCommand = new RetractIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeCommand = new ExtendIntakeCommand(m_intakeSubsystem);
  private final AngleClimbCommand m_angleClimbCommand = new AngleClimbCommand(m_climbSubsystem);
  private final StraightenClimbCommand m_straightenClimbCommand = new StraightenClimbCommand(m_climbSubsystem);


  public static JoystickButton intakeExtend, intakeRetract, intakeRun, intakeReverse, climbAngle, loadCatapult, launchCatapult, moveServo, moveClimbUp, moveClimbDown, climbStraight;
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
    intakeExtend = new JoystickButton(m_operatorController, HIDConstants.kStart);
    intakeRetract = new JoystickButton(m_operatorController, HIDConstants.kBack);
    intakeRun = new JoystickButton(m_operatorController, HIDConstants.kRB);
    intakeReverse = new JoystickButton(m_operatorController, HIDConstants.kX);
    loadCatapult = new JoystickButton(m_operatorController, HIDConstants.kLB);

    launchCatapult = new JoystickButton(m_operatorController, HIDConstants.kA);

    climbAngle = new JoystickButton(m_driverController, HIDConstants.kLB);
    climbStraight = new JoystickButton(m_driverController, HIDConstants.kRB);
    moveServo = new JoystickButton(m_driverController, HIDConstants.kB);
    moveClimbUp = new JoystickButton(m_driverController, HIDConstants.kY);
    moveClimbDown = new JoystickButton(m_driverController, HIDConstants.kA);

  
    intakeExtend.whenPressed(m_extendIntakeCommand);
    intakeRetract.whenPressed(m_retractIntakeCommand);
    intakeRun.whileHeld(m_runIntakeCommand);
    intakeReverse.whileHeld(m_reverseIntakeCommand);
    loadCatapult.whileHeld(m_loadCatapultCommand);

    moveServo.toggleWhenPressed(m_moveServoCommand);
    
    moveClimbUp.whileHeld(m_climbUpCommand);
    moveClimbDown.whileHeld(m_climbDownCommand);
    climbStraight.whenPressed(m_straightenClimbCommand);
    climbAngle.whenPressed(m_angleClimbCommand);


    launchCatapult.whileHeld(m_catapultCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An ExampleCommand will run in autonomous
    return m_autonomousDrive;
  }
}
