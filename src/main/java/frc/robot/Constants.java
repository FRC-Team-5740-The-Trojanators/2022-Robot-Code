// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final class HIDConstants 
    {
        /**
         * XBox Controller layout
         * kA = 1;
         * kB = 2;
         * kX = 3; 
         * kY = 4;
         * kLB = 5;
         * kRB = 6;
         * kSelect = 7;
         * kStart = 8;
         * kLeftStickPress = 9;
         * kRightStickPress = 10;
         */

        public static final int k_DriverControllerPort = 0;
        public static final int k_OperatorControllerPort = 1;
        public static final double kDeadBand = 0.1;
        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLB = 5;
        public static final int kRB = 6;
        public static final int kBack = 7;
        public static final int kStart = 8;
        public static final int kDL = 270;
        public static final int kDR = 90;
    }

    public static final class SwerveDriveModuleConstants
    {
        public static enum k_SwerveDriveModules
        {
            leftFront,
            rightFront,
            leftRear,
            rightRear,
        };

        // Distance between front and back wheels on robot; unit is meters
        public static final double k_WheelBase = Units.inchesToMeters(24.25);

        public static final double k_RobotRadius = Units.inchesToMeters(34.295) / 2; 

        public static final double k_wheelDiameter = Units.inchesToMeters(3.625); //meters

        /*TODO for all of these change when robot is characterized*/
        public static final double k_MaxTeleSpeed = Units.feetToMeters(16.3); //m/s 
        public static final double k_MaxAutoSpeed = Units.feetToMeters(16.3); //m/s
        public static final double k_MaxAcceleration = Units.feetToMeters(5); //m/s/s //TODO get real acceleration
        
        public static final double k_XYjoystickCoefficient = 1; //speed limiter
        public static final double k_MaxAngularSpeed = Units.feetToMeters(16.3) / k_RobotRadius; // 628; //rad/s TODO confirm
        public static final double k_RotCoefficient = 1; //speed limiter

        public static final double k_driveEncoderTicksPerRotation = 2048; 
        public static final double k_gearRatio = 6.75;
        public static final double k_temperatureLimit = 110.00;

        //public static double fieldCalibration = 0;

        //Angle offsets
        public static double leftFrontOffset =  16.35;
        public static double rightFrontOffset = 239.59;
        public static double leftRearOffset = 239.06 - 180;
        public static double rightRearOffset = 103.01;
    
        public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(k_WheelBase / 2, -k_WheelBase / 2),    // Left Front
            new Translation2d(k_WheelBase / 2, k_WheelBase / 2),   // Right Front
            new Translation2d(-k_WheelBase / 2, -k_WheelBase / 2),   // Left Rear
            new Translation2d(-k_WheelBase / 2, k_WheelBase / 2)); // Right Rear

    }

    /**
     * The CAN Bus device IDs for devices used with the Swerve Drive: motor controllers and encoders
    */
    public static final class CANBusIDs
    { 
        public static final int k_LeftFront_DriveMotor = 1; 
        public static final int leftFrontCANCoderId = 9; 
        public static final int k_LeftFront_SteeringMotor = 2;

        public static final int k_RightFront_DriveMotor = 3; 
        public static final int rightFrontCANCoderId = 10; 
        public static final int k_RightFront_SteeringMotor = 4; 

        public static final int k_LeftRear_DriveMotor = 5; 
        public static final int leftRearCANCoderId = 11; 
        public static final int k_LeftRear_SteeringMotor = 6;

        public static final int k_RightRear_DriveMotor = 7; 
        public static final int rightRearCANCoderId = 12; 
        public static final int k_RightRear_SteeringMotor = 8; 

        public static final int k_pigeonID = 13;
        public static final int k_intakeMotors = 15;
        public static final int k_holdMotor = 14;
        
        public static final int k_climbMotor = 16;
    }

    public static class DriveModulePIDValues
    {
        //TODO Set actual PID values
        public static double k_driveP = 0.028; 
        public static double k_driveI = 0.0;
        public static double k_driveD = 0.025;
        public static double k_driveFF = 1/Units.feetToMeters(16.3);
        public static final int k_ToleranceInTicks = 15; //TODO Figure out real value
    }

    public static class SteerModulePIDValues
    {
        public static double k_steerP = 5.0;
        public static double k_steerI = 0.0;
        public static double k_steerD = 0.50;

        public static final double k_steerDeadband = 0.02; // Deadband on the motor controller
        public static final int k_ToleranceInDegrees = 1;
    }
    
    public static final class PneumaticsConstants 
    {
        public static int k_left1ID = 4; 
        public static int k_left2ID = 2;
        public static int k_right1ID = 1;
        public static int k_right2ID = 6;

        public static int k_intakeForwardID = 5; 
        public static int k_intakeReverseID = 0;
        
        public static int k_climbForwardID = 3;
        public static int k_climbReverseID = 7;
    }
    
    public static final class RioInputs
    {
        public static int k_analogInputHighID = 0; //Analog Input
        public static int k_analogInputLowID = 1; //Analog Input

        public static int k_servoMotor = 0; //PWM output 
    }

    public static final class IntakeSubsystemConstants
    {
        public static final double k_intakeMotorSpeed = 0.6;
        public static final double k_intakeReverseMotorSpeed = -0.6;
        public static final double k_intakeStopMotorSpeed = 0.0;
        
        public static final double k_holdMotorSpeed = -.5;
        public static final double k_holdReverseMotorSpeed = 0.75;
        public static final double k_holdStopMotorSpeed = 0.0;
    }

    public static final class ClimbSubsystemConstants
    {
        public static final double k_servoAngledPosition = .5;
        public static final double k_servoStraightPosition = 0;

        /*
        Limit Switch Assignment: 
        Blue and Yellow is up
        Red/Purple and Black is down 
        */
    }


}
