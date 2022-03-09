// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import lib.LazyTalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.HIDConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.RioInputs;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new Climb. */
  private final DoubleSolenoid m_climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_climbForwardID, PneumaticsConstants.k_climbReverseID);
  //private final LazyTalonFX m_climbMotor = new LazyTalonFX(CANBusIDs.k_climbMotor);
  private final PWMTalonFX m_climbMotor = new PWMTalonFX(RioInputs.k_climbMotor);
 
  private final Servo m_servoMotor = new Servo(RioInputs.k_servoMotor);

  public ClimbSubsystem() 
  {
    //m_climbMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void setPower(double power)
  {
    m_climbMotor.set(power);
  }

  public void angleClimb()
  {
    m_climbSolenoid.set(Value.kForward);
  }

  public void straightClimb()
  {
    m_climbSolenoid.set(Value.kReverse);
  }

  public void stopClimb()
  {
    m_climbSolenoid.set(Value.kOff);
  }

  public void setServoMotor(double servoPower)
  {
   // m_servoMotor.set(servoPower);
    m_servoMotor.setPosition(servoPower);
  }

  public boolean getClimbAngle()
  {
    if( m_climbSolenoid.get() == Value.kForward)
    {
      return true;
    }
    else
    {
    return false;
    }
  }
}
