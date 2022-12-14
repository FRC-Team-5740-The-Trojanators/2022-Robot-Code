// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants;
import lib.LazyTalonFX;

public class DriveSubsystem extends SubsystemBase
{
  public PigeonIMU m_imu = new PigeonIMU(CANBusIDs.k_pigeonID);

  private SwerveModuleState[] m_states = new SwerveModuleState[]
  {
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0))
  }; 

  public SwerveModule[] modules = new SwerveModule[]
  {
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_LeftFront_DriveMotor), new LazyTalonFX(CANBusIDs.k_LeftFront_SteeringMotor), new CANCoder(CANBusIDs.leftFrontCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftFrontOffset)), // Left Front
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_RightFront_DriveMotor), new LazyTalonFX(CANBusIDs.k_RightFront_SteeringMotor), new CANCoder(CANBusIDs.rightFrontCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightFrontOffset)), // Right Front
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_LeftRear_DriveMotor), new LazyTalonFX(CANBusIDs.k_LeftRear_SteeringMotor), new CANCoder(CANBusIDs.leftRearCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftRearOffset)), // Left Rear
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_RightRear_DriveMotor), new LazyTalonFX(CANBusIDs.k_RightRear_SteeringMotor), new CANCoder(CANBusIDs.rightRearCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightRearOffset)), // Right Rear
  };
   
 public DriveSubsystem(boolean calibrateGyro) 
  {
    if(calibrateGyro) 
    {
      m_imu.setYaw(0); //recalibrates gyro offset
    }
        
    for(int i = 0; i < 4; i++)
    {
       modules[i].resetDriveEncoder();
    }

    m_imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 20);
  }

  public void teleDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
  {       
    m_states =
      SwerveDriveModuleConstants.kinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_imu.getYaw()))
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      SwerveDriveKinematics.desaturateWheelSpeeds(m_states, SwerveDriveModuleConstants.k_MaxTeleSpeed);
      for (int i = 0; i < 4; i++) 
      {
          SwerveModule module = modules[i];
          module.setDesiredState(m_states[i]);
      } 
  }
    
  public void resetEncoders()
  {
    modules[0].resetDriveEncoder();
    modules[1].resetDriveEncoder();
    modules[2].resetDriveEncoder();
    modules[3].resetDriveEncoder();
  }

  public void resetIMU()
  {
    m_imu.setYaw(0);
  }

  private final SwerveDriveOdometry m_odometry = 
      new SwerveDriveOdometry(SwerveDriveModuleConstants.kinematics, Rotation2d.fromDegrees(m_imu.getYaw()));

  @Override
  public void periodic() 
  {
  }
}