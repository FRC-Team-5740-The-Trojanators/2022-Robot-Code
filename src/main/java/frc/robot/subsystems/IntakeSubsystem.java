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
  //private final LazyVictorSPX m_intakeMotor = new LazyVictorSPX(CANBusIDs.k_intakeMotors);
  //private final LazyVictorSPX m_holdMotor = new LazyVictorSPX(CANBusIDs.k_holdMotor);
  private final PWMVictorSPX m_intakeMotors = new PWMVictorSPX(RioInputs.k_intakeMotors);
  private final PWMVictorSPX m_holdMotor = new PWMVictorSPX(RioInputs.k_holdMotor);


  public IntakeSubsystem() 
  {

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

  public void forwardIntakeMotors()
  {
    m_intakeMotors.set(IntakeSubsystemConstants.k_intakeMotorSpeed);
  }

  public void reverseIntakeMotors()
  {
    m_intakeMotors.set(IntakeSubsystemConstants.k_intakeReverseMotorSpeed);
  }

  public void stopIntakeMotors()
  {
    m_intakeMotors.set(IntakeSubsystemConstants.k_intakeStopMotorSpeed);
  }

  public void forwardHoldMotor()
  {
    m_holdMotor.set(IntakeSubsystemConstants.k_holdMotorSpeed);
  }
  
  public void reverseHoldMotor()
  {
    m_holdMotor.set(IntakeSubsystemConstants.k_holdReverseMotorSpeed);
  }
  
  public void stopHoldMotor()
  {
    m_holdMotor.set(IntakeSubsystemConstants.k_holdStopMotorSpeed);
  }
}
