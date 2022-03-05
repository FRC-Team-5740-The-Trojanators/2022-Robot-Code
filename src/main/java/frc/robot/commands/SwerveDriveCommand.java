// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.HIDConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
    private final DriveSubsystem drivetrain;
    private final XboxController controller;

                
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(20);//SlewRateLimiter(6);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(20);//SlewRateLimiter(6);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(15);

  public SwerveDriveCommand(DriveSubsystem drivetrain, XboxController controller)
  {
      this.drivetrain = drivetrain;
      this.controller = controller;

      addRequirements(drivetrain);
  }
  
  private double getJoystickWithDeadBand(double stickValue)
  {
      if(stickValue > HIDConstants.kDeadBand || stickValue < -HIDConstants.kDeadBand)
      {
          if(stickValue < 0)
          {
              return stickValue = -Math.pow(stickValue, 2);
          }
          else
          {
              return stickValue = Math.pow(stickValue, 2);
          }
      } 
      else 
      {
          return 0;
      }
  } 
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      // Get the x speed.
      final var xSpeed =
         -xspeedLimiter.calculate(getJoystickWithDeadBand(controller.getLeftY())
          * SwerveDriveModuleConstants.k_MaxTeleSpeed * SwerveDriveModuleConstants.k_XYjoystickCoefficient);

      final var ySpeed =
        yspeedLimiter.calculate(getJoystickWithDeadBand(controller.getLeftX())
        * SwerveDriveModuleConstants.k_MaxTeleSpeed * SwerveDriveModuleConstants.k_XYjoystickCoefficient);
     
      final var rot =
        rotLimiter.calculate(getJoystickWithDeadBand(controller.getRightX())
        * SwerveDriveModuleConstants.k_MaxAngularSpeed * SwerveDriveModuleConstants.k_RotCoefficient);

      drivetrain.teleDrive(xSpeed, ySpeed, rot, false);
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
