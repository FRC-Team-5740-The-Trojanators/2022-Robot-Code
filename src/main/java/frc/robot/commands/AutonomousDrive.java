// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoChooserSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import lib.swerve.SwervePath;
import lib.swerve.SwervePathController;
import frc.robot.RobotContainer;


public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  private final DriveSubsystem m_driveSubsystem;
  private final AutoChooserSubsystem m_autoChooser;

  private Timer timer;
  SwervePath path;
  SwervePathController pathController;
  double lastTime;
  boolean ignoreHeading;

  DigitalInput dInput = new DigitalInput(1);


  public AutonomousDrive(DriveSubsystem driveSubsystem, AutoChooserSubsystem autoChooser) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_autoChooser = autoChooser;

    if(dInput.get() == true)
    {
      this.path = SwervePath.fromCSV("Default taxi");  
    } else if (dInput.get() == false) {
      this.path = SwervePath.fromCSV("New Path");  
    }


    addRequirements(driveSubsystem, autoChooser);
   

    this.timer = new Timer();
  
    this.ignoreHeading = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer = new Timer();
    timer.reset();
    timer.start(); 
    SwervePath.State initialState = path.getInitialState();

    m_driveSubsystem.resetOdometry(new Pose2d(m_driveSubsystem.getPoseMeters().getTranslation(), initialState.getRotation()));
    m_driveSubsystem.resetEncoders();
    
    lastTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double time = timer.get();
    SwervePath.State desiredState = path.sample(time);

    if(ignoreHeading) desiredState.rotation = new Rotation2d(0);

    m_driveSubsystem.autoDrive(desiredState.getVelocity() * desiredState.getRotation().getCos(), desiredState.getVelocity() * desiredState.getRotation().getSin(), 0, false);

    lastTime = time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    timer.stop();
    m_driveSubsystem.autoDrive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return timer.hasElapsed(path.getRuntime());
  }
}