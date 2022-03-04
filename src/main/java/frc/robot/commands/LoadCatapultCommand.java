// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadCatapultCommand extends CommandBase {
  /** Creates a new IntakeReverseCommand. */
  private final IntakeSubsystem m_intake;
  private boolean m_isFinished = false;
  
  public LoadCatapultCommand(IntakeSubsystem intake)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_intake.reverseHoldMotor();
    m_intake.forwardIntakeMotors();
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_intake.reverseHoldMotor();
    m_intake.forwardIntakeMotors();
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
  public boolean isFinished() 
  {
    return m_isFinished;
  }
}
