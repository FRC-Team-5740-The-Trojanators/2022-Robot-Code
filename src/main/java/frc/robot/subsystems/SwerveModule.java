// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.nio.charset.CharsetEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/**
 * A Swerve Module consists of a drive motor, a steering motor, and encoders to
 * provide feedback on the state of those motors. This code provides accessors
 * to those motors' controllers and encoders, as well as defining the feedback
 * loops used to enhance their control.
 */
public class SwerveModule 
{

    private Talon m_driveMotor;
    private Talon m_angleMotor;
    private Rotation2d m_offset;

    private CANCoder m_moduleSteeringEncoder;
    //TODO make a m_driveEncoder

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     */
    public SwerveModule(Talon driveMotor, Talon angleMotor, CANCoder canCoder, Rotation2d offset)
    {
        m_driveMotor = driveMotor;
        m_angleMotor = angleMotor;
        m_moduleSteeringEncoder = canCoder;
        m_offset = offset;

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = m_offset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoder.configAllSettings(canCoderConfiguration);
    }
}
