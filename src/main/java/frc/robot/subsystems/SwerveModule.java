// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveModulePIDValues;
import frc.robot.Constants.SteerModulePIDValues;
import frc.robot.Constants.SwerveDriveModuleConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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

    private TalonFX m_driveMotor;
    private TalonFX m_angleMotor;
    private Rotation2d m_offset;

    private CANCoder m_moduleSteeringEncoder;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     */
    public SwerveModule(TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, Rotation2d offset)
    {
        m_driveMotor = driveMotor;
        m_angleMotor = angleMotor;
        m_moduleSteeringEncoder = canCoder;
        m_offset = offset;

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = m_offset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoder.configAllSettings(canCoderConfiguration);

        m_angleMotor.configFactoryDefault();
        m_angleMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration angleTalonConfig = new TalonFXConfiguration();
    
        angleTalonConfig.slot0.allowableClosedloopError = SteerModulePIDValues.k_ToleranceInDegrees;
        angleTalonConfig.remoteFilter0.remoteSensorDeviceID = m_moduleSteeringEncoder.getDeviceID();
        angleTalonConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleTalonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        
        angleTalonConfig.slot0.kP = SteerModulePIDValues.k_steerP;
        angleTalonConfig.slot0.kI = SteerModulePIDValues.k_steerI;
        angleTalonConfig.slot0.kD = SteerModulePIDValues.k_steerD;
        angleTalonConfig.slot0.kF = SteerModulePIDValues.k_steerFF;

        m_angleMotor.configAllSettings(angleTalonConfig);

        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    
        driveTalonConfig.slot0.allowableClosedloopError = DriveModulePIDValues.k_ToleranceInTicks;
        driveTalonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        
        driveTalonConfig.slot0.kP = DriveModulePIDValues.k_driveP;
        driveTalonConfig.slot0.kI = DriveModulePIDValues.k_driveI;
        driveTalonConfig.slot0.kD = DriveModulePIDValues.k_driveD;
        driveTalonConfig.slot0.kF = DriveModulePIDValues.k_driveFF;
      
        m_driveMotor.configAllSettings(driveTalonConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState)
    {      
       //Steering Motor Calc
        Rotation2d currentRotation = getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        Rotation2d rotationDelta = state.angle.minus(currentRotation); //takes our current rotatation and subtracts the last state rotation
       
        double deltaTicks = calculateDeltaTicks(rotationDelta);
        double currentTicks = calculateCurrentTicks();
        double desiredTicks = currentTicks + deltaTicks;
        //double setAngle = m_steeringPIDController.calculate(currentTicks, desiredTicks);

        m_angleMotor.set(TalonFXControlMode.Position, filterAngleMotorDeadband(desiredTicks));;
        
        //m_driverPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        m_driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / SwerveDriveModuleConstants.k_MaxTeleSpeed);
    }

    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(m_moduleSteeringEncoder.getAbsolutePosition());
    }

    public double calculateDeltaTicks(Rotation2d rotationDelta) 
    {
        return (rotationDelta.getDegrees() / 360) * SwerveDriveModuleConstants.k_driveEncoderTicksPerRotation;
    }

    public double calculateCurrentTicks() {
        double currentTicks = m_moduleSteeringEncoder.getPosition() / m_moduleSteeringEncoder.configGetFeedbackCoefficient();
        return currentTicks;
    }

    public double filterAngleMotorDeadband(double setAngle)
    {
        if(Math.abs(setAngle) > SteerModulePIDValues.k_steerDeadband)
        {
           return setAngle;
        } 
        return 0.0;
    }

    public void resetDriveEncoder()
    {
        m_driveMotor.setSelectedSensorPosition(0);
        m_moduleSteeringEncoder.setPosition(0);  
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), Rotation2d.fromDegrees(m_moduleSteeringEncoder.getPosition()));
    }

    public double getDriveVelocity()
    {
        return m_driveMotor.getMotorOutputPercent() * SwerveDriveModuleConstants.k_MaxTeleSpeed;
    }

    public double getRotationDegrees()
    {
        return m_moduleSteeringEncoder.getPosition();
    }

    public void adjustPIDValues()
    {
        TalonFXConfiguration drivePIDTalonConfig = new TalonFXConfiguration();

        SmartDashboard.putNumber("Drive P", DriveModulePIDValues.k_driveP);
        SmartDashboard.putNumber("Drive I", DriveModulePIDValues.k_driveI);
        SmartDashboard.putNumber("Drive D", DriveModulePIDValues.k_driveD);
        SmartDashboard.putNumber("Drive FF", DriveModulePIDValues.k_driveFF);
    
        SmartDashboard.putNumber("Steer P", SteerModulePIDValues.k_steerP);
        SmartDashboard.putNumber("Steer I", SteerModulePIDValues.k_steerI);
        SmartDashboard.putNumber("Steer D", SteerModulePIDValues.k_steerD);


        double p = SmartDashboard.getNumber("P Gain", DriveModulePIDValues.k_driveP);
        double i = SmartDashboard.getNumber("I Gain", DriveModulePIDValues.k_driveI);
        double d = SmartDashboard.getNumber("D Gain", DriveModulePIDValues.k_driveD);
        double ff = SmartDashboard.getNumber("FF Gain", DriveModulePIDValues.k_driveFF);
    
        if((p != DriveModulePIDValues.k_driveP)) { drivePIDTalonConfig.slot0.kP = p; DriveModulePIDValues.k_driveP = p; }
        if((i != DriveModulePIDValues.k_driveI)) { drivePIDTalonConfig.slot0.kI = i; DriveModulePIDValues.k_driveI = i; }
        if((d != DriveModulePIDValues.k_driveD)) { drivePIDTalonConfig.slot0.kD = d; DriveModulePIDValues.k_driveD = d; }
        if((ff != DriveModulePIDValues.k_driveFF)) { drivePIDTalonConfig.slot0.kF = ff; DriveModulePIDValues.k_driveFF = ff; }

        if((p != DriveModulePIDValues.k_driveP) || (i != DriveModulePIDValues.k_driveI) || (d != DriveModulePIDValues.k_driveD) || (ff != DriveModulePIDValues.k_driveFF))
        {
            m_driveMotor.configAllSettings(drivePIDTalonConfig);
        }

        TalonFXConfiguration steerPIDTalonConfig = new TalonFXConfiguration();

        double pS = SmartDashboard.getNumber("P Gain", SteerModulePIDValues.k_steerP);
        double iS = SmartDashboard.getNumber("I Gain", SteerModulePIDValues.k_steerI);
        double dS = SmartDashboard.getNumber("D Gain", SteerModulePIDValues.k_steerD);
    
        if((pS != SteerModulePIDValues.k_steerP)) { steerPIDTalonConfig.slot0.kP = p; SteerModulePIDValues.k_steerP = p; }
        if((iS != SteerModulePIDValues.k_steerI)) { steerPIDTalonConfig.slot0.kI = i; SteerModulePIDValues.k_steerI = i; }
        if((dS != SteerModulePIDValues.k_steerD)) { steerPIDTalonConfig.slot0.kD = d; SteerModulePIDValues.k_steerD = d; }

        if((p != SteerModulePIDValues.k_steerP) || (i != SteerModulePIDValues.k_steerI) || (d != SteerModulePIDValues.k_steerD))
        {
            m_driveMotor.configAllSettings(drivePIDTalonConfig);
        }
    }

}
