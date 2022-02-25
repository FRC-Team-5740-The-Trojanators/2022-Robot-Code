// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.RioInputs;

public class CatapultSubsystem extends SubsystemBase {
  /** Creates a new CatapultSubsystem. */
  private final Solenoid m_Left1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_left1ID);
  private final Solenoid m_Left2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_left2ID);
  private final Solenoid m_Right1 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_right1ID);
  private final Solenoid m_Right2 = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_right2ID);

  private final AnalogInput m_analogInputHigh = new AnalogInput(RioInputs.k_analogInputHighID);
  private final AnalogInput m_analogInputLow = new AnalogInput(RioInputs.k_analogInputLowID);


  //(250*(averageVolts/5.0))-25; - this is the equation for analong pressure reading

  public CatapultSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double averageVolts = m_analogInputHigh.getVoltage(); 
    SmartDashboard.putNumber("High Pressure", (250*(averageVolts/5.0))-25);

    averageVolts = m_analogInputLow.getVoltage(); 
    SmartDashboard.putNumber("Low Pressure", (250*(averageVolts/5.0))-25);
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
