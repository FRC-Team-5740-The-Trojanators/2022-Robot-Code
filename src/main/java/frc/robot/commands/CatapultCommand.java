// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

public class CatapultCommand extends CommandBase {
  /** Creates a new CatapultCommand. */
  private final CatapultSubsystem m_catapult;
  public CatapultCommand(CatapultSubsystem catapult) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_catapult = catapult;
    addRequirements(catapult);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_catapult.shootCatapult();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_catapult.catapultDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
