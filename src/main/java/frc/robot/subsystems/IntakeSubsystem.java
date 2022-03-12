// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import lib.LazyVictorSPX;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.RioInputs;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_intakeForwardID, PneumaticsConstants.k_intakeReverseID);
  private final PWMVictorSPX m_intakeMotor;
  private final PWMVictorSPX m_holdMotor;

  public IntakeSubsystem() 
  {
    m_intakeMotor = new PWMVictorSPX(RioInputs.k_intakeMotors);
    m_holdMotor = new PWMVictorSPX(RioInputs.k_holdMotor);

   // m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
   // m_holdMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);

  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public boolean getIntakeState()
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
      m_intakeSolenoid.set(Value.kReverse);
  }

  public void retractIntake()
  {
    m_intakeSolenoid.set(Value.kForward);
  }

  public void stopIntakeSolenoid()
  {
    m_intakeSolenoid.set(Value.kOff);
  }

  public void forwardIntakeMotors()
  {
    m_intakeMotor.set(IntakeSubsystemConstants.k_intakeMotorSpeed);
  }

  public void reverseIntakeMotors()
  {
    m_intakeMotor.set( IntakeSubsystemConstants.k_intakeReverseMotorSpeed);
  }

  public void stopIntakeMotors()
  {
    m_intakeMotor.set( IntakeSubsystemConstants.k_intakeStopMotorSpeed);
  }

  public void forwardHoldMotor()
  {
    m_holdMotor.set( IntakeSubsystemConstants.k_holdMotorSpeed);
  }
  
  public void reverseHoldMotor()
  {
    m_holdMotor.set( IntakeSubsystemConstants.k_holdReverseMotorSpeed);
  }
  
  public void stopHoldMotor()
  {
    m_holdMotor.set( IntakeSubsystemConstants.k_holdStopMotorSpeed);
  }
}
