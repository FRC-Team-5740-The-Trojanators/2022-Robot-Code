// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LeftCatapultCommand extends CommandBase {
  /** Creates a new CatapultCommand. */
  private final CatapultSubsystem m_catapult;
  private final IntakeSubsystem m_intake;
  private boolean m_isFinished = false;
  Timer m_timer = new Timer();

  public LeftCatapultCommand(CatapultSubsystem catapult, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_catapult = catapult;
    m_intake = intake;
    addRequirements(catapult, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
   {
    m_timer.reset();
    m_timer.start();
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(m_intake.getIntakeState() == false)
    {
      m_catapult.shootRight();
    }
    m_isFinished = m_timer.advanceIfElapsed(1.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_catapult.catapultDown();
    m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
