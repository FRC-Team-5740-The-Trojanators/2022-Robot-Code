// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.HIDConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.RioInputs;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new Climb. */
  private final DoubleSolenoid m_climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.k_climbForwardID, PneumaticsConstants.k_climbReverseID);
  private final TalonFX m_climbMotor = new TalonFX(CANBusIDs.k_climbMotor);
 
  private final Servo m_servoMotor = new Servo(RioInputs.k_servoMotor);
  private final DigitalInput topLimitSwitch = new DigitalInput(RioInputs.k_topLimitSwitchID);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(RioInputs.k_bottomLimitSwitchID);

  public ClimbSubsystem() 
  {
    m_climbMotor.configFactoryDefault();
    m_climbMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public double getClimbDistance() 
  {
		return m_climbMotor.getSelectedSensorPosition();
  }

  public void setPower(double power)
  {
    m_climbMotor.set(ControlMode.PercentOutput, power);
  }

  public boolean topLimitSwitchBoolean()
  {
    return topLimitSwitch.get();  //true means the limit switch has been tripped
  }

  public boolean bottomLimitSwitchBoolean()
  {
    return bottomLimitSwitch.get();  //true means the limit switch has been tripped
  }

  public double getJoystickWithDeadBand(double stickValue)
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
    m_servoMotor.set(servoPower);
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
