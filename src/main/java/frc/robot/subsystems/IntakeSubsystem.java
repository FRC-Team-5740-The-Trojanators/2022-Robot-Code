// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
  private final VictorSPX m_intakeMotor;
  private final VictorSPX m_holdMotor;

  public IntakeSubsystem() 
  {
    m_intakeMotor = new VictorSPX(CANBusIDs.k_intakeMotors);
    m_holdMotor = new VictorSPX(CANBusIDs.k_holdMotor);
    m_intakeSolenoid.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isIntakeOut()
  {
    if(m_intakeSolenoid.get() == Value.kForward)
    {
      return true;
    } else
    {
      return false;
    }
  }

  public void extendIntake()
  {
      m_intakeSolenoid.set(Value.kForward);
  }

  public void retractIntake()
  {
    m_intakeSolenoid.set(Value.kReverse);
  }

  public void startIntakeMotors()
  {
    m_intakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_intakeMotorSpeed);
  }

  public void reverseIntakeMotors()
  {
    m_intakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_intakeReverseMotorSpeed);
  }

  public void stopIntakeMotors()
  {
    m_intakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_intakeStopMotorSpeed);
  }

  public void startHoldMotor()
  {
    m_holdMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_holdMotorSpeed);
  }
  
  public void reverseHoldMotor()
  {
    m_holdMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_holdReverseMotorSpeed);
  }
  
  public void stopHoldMotor()
  {
    m_holdMotor.set(VictorSPXControlMode.PercentOutput, IntakeSubsystemConstants.k_holdStopMotorSpeed);
  }
}
