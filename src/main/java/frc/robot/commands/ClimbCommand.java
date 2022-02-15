// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
  /** Creates a new ClimbCommand. */

  private final ClimbSubsystem m_climb;
  private double power = 0;
  private final XboxController m_controller;
  private boolean m_isFinished = false; 
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(20);//SlewRateLimiter(6);


  public ClimbCommand(ClimbSubsystem climb, XboxController controller)
  {
    m_climb = climb;
    m_controller = controller;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    final var ySpeed = -yspeedLimiter.calculate(m_climb.getJoystickWithDeadBand(m_controller.getLeftY()));
    
    if (m_climb.topLimitSwitchBoolean() != true && m_climb.bottomLimitSwitchBoolean() != true) 
    {
      m_climb.setPower(ySpeed);
    } 
    else 
    {
      m_climb.setPower(power);
      m_isFinished = true;
    }

    if(m_climb.getClimbAngle())
    {
      m_climb.setServoMotor(ClimbSubsystemConstants.k_servoAngledPosition);
    }
    else
    {
      m_climb.setServoMotor(ClimbSubsystemConstants.k_servoStraightPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_climb.setPower(power);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    m_climb.setPower(power);
    return m_isFinished;
  }
}
