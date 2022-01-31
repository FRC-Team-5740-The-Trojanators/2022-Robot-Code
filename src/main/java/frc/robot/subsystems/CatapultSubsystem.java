// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class CatapultSubsystem extends SubsystemBase {
  /** Creates a new CatapultSubsystem. */
  private final Solenoid m_Left1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_left1ID);
  private final Solenoid m_Left2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_left2ID);
  private final Solenoid m_Right1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_right1ID);
  private final Solenoid m_Right2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_right2ID);
  public CatapultSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shootCatapult()
  {
      m_Left1.set(true);
      m_Left2.set(true);
      m_Right1.set(true);
      m_Right2.set(true);
  }
  
  public void catapultDown()
  {
      m_Left1.set(false);
      m_Left2.set(false);
      m_Right1.set(false);
      m_Right2.set(false);
  }
}
