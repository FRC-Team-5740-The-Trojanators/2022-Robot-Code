// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbDownCommand extends CommandBase {
  /** Creates a new ClimbCommand. */

  private final ClimbSubsystem m_climb;
  private final IntakeSubsystem m_intake;
  private double power = -.5;
  private boolean m_isFinished = false; 
  private Timer m_timer;

  public ClimbDownCommand(ClimbSubsystem climb, IntakeSubsystem intake)
  {
    m_climb = climb;
    m_intake = intake;
    this.m_timer = new Timer();
    addRequirements(climb, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_timer.start();
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    m_climb.setPower(power);

    if( m_climb.getRevLimitSwitch() == 1)
    {
      m_intake.extendIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_climb.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return m_isFinished;
  }
}
