// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class AngleClimbCommand extends CommandBase {
  /** Creates a new AngleClimb. */
  private final ClimbSubsystem m_climb;
  private boolean m_isFinished;

  public AngleClimbCommand(ClimbSubsystem climb) 
  {
    m_climb = climb;
    addRequirements(m_climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_climb.angleClimb();
    m_isFinished = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_climb.angleClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
