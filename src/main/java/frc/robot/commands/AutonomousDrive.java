// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import lib.swerve.SwervePath;
import lib.swerve.SwervePathController;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  private final DriveSubsystem m_driveSubsystem;

  private Timer timer;
  SwervePath path;
  SwervePathController pathController;
  double lastTime;
  boolean ignoreHeading;

  public AutonomousDrive(DriveSubsystem driveSubsystem) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    m_driveSubsystem = driveSubsystem;

    // SendableChooser<String> auto = new SendableChooser<String>();
    //  auto.addOption("Default taxi", "Default taxi");
    //  auto.addOption("Position 1", "Position 1");
    //  auto.addOption("Position 2", "Position 2");
    //  auto.addOption("Position 3", "Position 3");
    //  auto.addOption("Position 4", "Position 4");

    // Shuffleboard.getTab("user tab").add(auto);

    this.timer = new Timer();
    //this.path = SwervePath.fromCSV(auto.getSelected());  
     this.path = SwervePath.fromCSV("Default taxi");    
  
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