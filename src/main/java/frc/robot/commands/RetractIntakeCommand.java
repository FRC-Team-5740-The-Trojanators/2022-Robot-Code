// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends CommandBase {
  /** Creates a new FlipIntakeCommand. */
  IntakeSubsystem m_intake;
  private boolean m_isFinished = false;
  Timer m_timer = new Timer();


  public RetractIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
   m_timer.reset();
   m_timer.start();
   m_intake.retractIntake();
   m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_intake.retractIntake();
    m_isFinished = m_timer.advanceIfElapsed(1.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //m_intake.stopIntakeSolenoid();
    m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  m_isFinished;
  }
}
