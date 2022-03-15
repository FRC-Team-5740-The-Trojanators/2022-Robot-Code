// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HIDConstants;
import frc.robot.commands.AngleClimbCommand;
import frc.robot.commands.AutoCatapultCommand;
import frc.robot.commands.AutoPosition1;
import frc.robot.commands.AutoPosition2;
import frc.robot.commands.AutoPosition3;
import frc.robot.commands.AutoPosition4;
import frc.robot.commands.AutoDefaultTaxi;
import frc.robot.commands.AutoLoadCatapultCommand;
import frc.robot.commands.AutoPos4Part2;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.LoadCatapultCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.AutoRunIntakeCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.StraightenClimbCommand;
import frc.robot.commands.MoveServoCommand;
import frc.robot.commands.MoveServoDownCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

 // private final ExampleCommand m_exampleCommand = new ExampleCommand();
  private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);
  private final RunIntakeCommand m_runIntakeCommand = new RunIntakeCommand(m_intakeSubsystem);
  private final LoadCatapultCommand m_loadCatapultCommand = new LoadCatapultCommand(m_intakeSubsystem);
  private final ReverseIntakeCommand m_reverseIntakeCommand = new ReverseIntakeCommand(m_intakeSubsystem);
  private final CatapultCommand m_catapultCommand = new CatapultCommand(m_catapultSubsystem, m_intakeSubsystem);
  private final ClimbUpCommand m_climbUpCommand = new ClimbUpCommand(m_climbSubsystem);
  private final ClimbDownCommand m_climbDownCommand = new ClimbDownCommand(m_climbSubsystem);
  private final MoveServoCommand m_moveServoCommand = new MoveServoCommand(m_climbSubsystem);
  private final MoveServoDownCommand m_moveServoDownCommand = new MoveServoDownCommand(m_climbSubsystem);
  private final RetractIntakeCommand m_retractIntakeCommand = new RetractIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeCommand = new ExtendIntakeCommand(m_intakeSubsystem);
  private final AngleClimbCommand m_angleClimbCommand = new AngleClimbCommand(m_climbSubsystem);
  private final StraightenClimbCommand m_straightenClimbCommand = new StraightenClimbCommand(m_climbSubsystem);
 
  private final AutoDefaultTaxi m_autoDefaultTaxi = new AutoDefaultTaxi(m_driveSubsystem);
  private final AutoPosition1 m_autoPath1 = new AutoPosition1(m_driveSubsystem);
  private final AutoPosition2 m_autoPath2 = new AutoPosition2(m_driveSubsystem);
  private final AutoPosition3 m_autoPath3 = new AutoPosition3(m_driveSubsystem);
  private final AutoPosition4 m_autoPath4 = new AutoPosition4(m_driveSubsystem);

  private final AutoPosition4 m_autoPathThreeBall1 = new AutoPosition4(m_driveSubsystem);
  private final AutoPos4Part2 m_autoPathThreeBall2 = new AutoPos4Part2(m_driveSubsystem);

  private final ExtendIntakeCommand m_extendIntakeAuto = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto2 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto3 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto4 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto5 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto6 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto7 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto8 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto9 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto10 = new ExtendIntakeCommand(m_intakeSubsystem);
  private final ExtendIntakeCommand m_extendIntakeAuto11 = new ExtendIntakeCommand(m_intakeSubsystem);

  private final RetractIntakeCommand m_retractIntakeAuto = new RetractIntakeCommand(m_intakeSubsystem);
  private final RetractIntakeCommand m_retractIntakeAuto2 = new RetractIntakeCommand(m_intakeSubsystem);
  private final RetractIntakeCommand m_retractIntakeAuto3 = new RetractIntakeCommand(m_intakeSubsystem);
  private final RetractIntakeCommand m_retractIntakeAuto4 = new RetractIntakeCommand(m_intakeSubsystem);
  private final RetractIntakeCommand m_retractIntakeAuto5 = new RetractIntakeCommand(m_intakeSubsystem);
  private final RetractIntakeCommand m_retractIntakeAuto6 = new RetractIntakeCommand(m_intakeSubsystem);

  private final AutoRunIntakeCommand m_runIntakeAuto = new AutoRunIntakeCommand(m_intakeSubsystem);
  private final AutoRunIntakeCommand m_runIntakeAuto2 = new AutoRunIntakeCommand(m_intakeSubsystem);
  private final AutoRunIntakeCommand m_runIntakeAuto3 = new AutoRunIntakeCommand(m_intakeSubsystem);
  private final AutoRunIntakeCommand m_runIntakeAuto4 = new AutoRunIntakeCommand(m_intakeSubsystem);
  private final AutoRunIntakeCommand m_runIntakeAuto5 = new AutoRunIntakeCommand(m_intakeSubsystem);
  private final AutoRunIntakeCommand m_runIntakeAuto6 = new AutoRunIntakeCommand(m_intakeSubsystem);

  private final AutoLoadCatapultCommand m_loadCatapultAuto = new AutoLoadCatapultCommand(m_intakeSubsystem);
  private final AutoLoadCatapultCommand m_loadCatapultAuto2 = new AutoLoadCatapultCommand(m_intakeSubsystem);
  private final AutoLoadCatapultCommand m_loadCatapultAuto3 = new AutoLoadCatapultCommand(m_intakeSubsystem);
  private final AutoLoadCatapultCommand m_loadCatapultAuto4 = new AutoLoadCatapultCommand(m_intakeSubsystem);
  private final AutoLoadCatapultCommand m_loadCatapultAuto5 = new AutoLoadCatapultCommand(m_intakeSubsystem);
  private final AutoLoadCatapultCommand m_loadCatapultAuto6 = new AutoLoadCatapultCommand(m_intakeSubsystem);

  private final AutoCatapultCommand m_catapultAuto = new AutoCatapultCommand(m_catapultSubsystem, m_intakeSubsystem);
  private final AutoCatapultCommand m_catapultAuto2 = new AutoCatapultCommand(m_catapultSubsystem, m_intakeSubsystem);
  private final AutoCatapultCommand m_catapultAuto3 = new AutoCatapultCommand(m_catapultSubsystem, m_intakeSubsystem);
  private final AutoCatapultCommand m_catapultAuto4 = new AutoCatapultCommand(m_catapultSubsystem, m_intakeSubsystem);
  private final AutoCatapultCommand m_catapultAuto5 = new AutoCatapultCommand(m_catapultSubsystem, m_intakeSubsystem);
  private final AutoCatapultCommand m_catapultAuto6 = new AutoCatapultCommand(m_catapultSubsystem, m_intakeSubsystem);

  //private final SequentialCommandGroup m_pathWithIntake = new SequentialCommandGroup(m_extendIntakeAuto, m_autoPath1);
  
  private final ParallelCommandGroup m_position1Parallel = new ParallelCommandGroup(m_runIntakeAuto, m_autoPath1);
  private final ParallelCommandGroup m_position2Parallel = new ParallelCommandGroup(m_runIntakeAuto2, m_autoPath2);
  private final ParallelCommandGroup m_position3Parallel = new ParallelCommandGroup(m_runIntakeAuto3, m_autoPath3);
  private final ParallelCommandGroup m_position4Parallel = new ParallelCommandGroup(m_runIntakeAuto4, m_autoPath4);
  private final ParallelCommandGroup m_threeBallPart1Parallel = new ParallelCommandGroup(m_runIntakeAuto5, m_autoPathThreeBall1);
  private final ParallelCommandGroup m_threeBallPart2Parallel = new ParallelCommandGroup(m_runIntakeAuto6, m_autoPathThreeBall2);

  private final SequentialCommandGroup m_position1Sequential = new SequentialCommandGroup(m_extendIntakeAuto, m_position1Parallel, m_retractIntakeAuto, m_loadCatapultAuto, m_extendIntakeAuto2, m_catapultAuto);
  private final SequentialCommandGroup m_position2Sequential = new SequentialCommandGroup(m_extendIntakeAuto3, m_position2Parallel, m_retractIntakeAuto2, m_loadCatapultAuto2, m_extendIntakeAuto4, m_catapultAuto2);
  private final SequentialCommandGroup m_position3Sequential = new SequentialCommandGroup(m_extendIntakeAuto5, m_position3Parallel, m_retractIntakeAuto3, m_loadCatapultAuto3, m_extendIntakeAuto6, m_catapultAuto3);
  private final SequentialCommandGroup m_position4Sequential = new SequentialCommandGroup(m_extendIntakeAuto7, m_position4Parallel, m_retractIntakeAuto4, m_loadCatapultAuto4, m_extendIntakeAuto8, m_catapultAuto4);
  private final SequentialCommandGroup m_threeBallPart1 = new SequentialCommandGroup(m_extendIntakeAuto9, m_threeBallPart1Parallel, m_retractIntakeAuto5, m_loadCatapultAuto5, m_extendIntakeAuto10, m_catapultAuto5);
  private final SequentialCommandGroup m_threeBall = new SequentialCommandGroup(m_threeBallPart1, m_threeBallPart2Parallel, m_retractIntakeAuto6, m_loadCatapultAuto6, m_extendIntakeAuto11, m_catapultAuto6);

//private final SequentialCommandGroup m_testIntakeStepSequential = new SequentialCommandGroup(m_loadCatapultAuto, m_extendIntakeAuto, m_catapultAuto);

  public static JoystickButton intakeExtend, intakeRetract, intakeRun, intakeReverse, climbAngle, loadCatapult, launchCatapult, moveClimbUp, moveClimbDown, climbStraight;
  public static POVButton  moveServo, moveServoDown;
  SendableChooser<CommandBase> auto = new SendableChooser<CommandBase>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the button bindings
    configureButtonBindings();
    configChooser();
    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_driveSubsystem.resetIMU();

  }

  private void configChooser()
  {
    //configure the auto command chooser
    auto.addOption("Position 1", m_position1Sequential);
    auto.addOption("Position 2", m_position2Sequential);
    auto.addOption("Position 3", m_position3Sequential);
    auto.addOption("Position 4", m_position4Sequential);
    auto.addOption("ThreeBall", m_threeBall);

    auto.setDefaultOption("Default taxi", m_autoDefaultTaxi);

    SmartDashboard.putData(auto);
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
    moveServo = new POVButton(m_driverController, HIDConstants.kDU);
    moveServoDown = new POVButton(m_driverController, HIDConstants.kDD);
    moveClimbUp = new JoystickButton(m_driverController, HIDConstants.kY);
    moveClimbDown = new JoystickButton(m_driverController, HIDConstants.kA);

    intakeExtend.whenActive(m_extendIntakeCommand);
    intakeRetract.whenPressed(m_retractIntakeCommand);
    intakeRun.whileHeld(m_runIntakeCommand);
    intakeReverse.whileHeld(m_reverseIntakeCommand);
    loadCatapult.whileHeld(m_loadCatapultCommand);

    moveServo.whenPressed(m_moveServoCommand);
    moveServoDown.whenPressed(m_moveServoDownCommand);
    
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
    return auto.getSelected();
    //return m_autoDefaultTaxi;
  }
}
