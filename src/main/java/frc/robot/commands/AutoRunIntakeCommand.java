// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRunIntakeCommand extends CommandBase {
  /** Creates a new IntakeRunCommand. */
  private final IntakeSubsystem m_intake;
  private boolean m_isFinished = false;
  private Timer m_timer = new Timer();

  public AutoRunIntakeCommand(IntakeSubsystem intake)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_intake.forwardIntakeMotors();
    m_intake.forwardHoldMotor();
    m_isFinished = false;
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_timer.start();
    m_intake.forwardIntakeMotors();
    m_intake.forwardHoldMotor();

    if(m_timer.get() >= 4)
    {
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_intake.stopIntakeMotors();
    m_intake.stopHoldMotor();

    m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
