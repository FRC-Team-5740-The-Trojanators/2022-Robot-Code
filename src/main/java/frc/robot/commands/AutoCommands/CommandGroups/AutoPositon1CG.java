// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.LoadCatapultCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.RunIntakeCommand;
// import frc.robot.commands.AutoCommands.AutonomousPathFollow;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPositon1CG extends SequentialCommandGroup {
  /** Creates a new AutoShootingCG. */
  IntakeSubsystem m_intake;
  CatapultSubsystem m_catapult;
  DriveSubsystem m_drive;
  public AutoPositon1CG() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands
    (
      //new ExtendIntakeCommand(m_intake),
      //new ParallelCommandGroup
      //   (new AutonomousPathFollow(m_drive, "Position 1"), 
      //    new RunIntakeCommand(m_intake)),
      new RetractIntakeCommand(m_intake),
      new LoadCatapultCommand(m_intake),
      new ExtendIntakeCommand(m_intake),
      new CatapultCommand(m_catapult, m_intake)
    );
  }
}
