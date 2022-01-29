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
        public static final double kDeadBand = 0.05;
        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLB = 5;
        public static final int kRB = 6;
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

        // Distance between centers of right and left wheels on robot; unit is meters
        public static final double k_TrackWidth = 0.00000000001; //TODO Measure Track Width

        // Distance between front and back wheels on robot; unit is meters
        public static final double k_WheelBase = 0.000000000001; //TODO Measure Wheel Base

        public static final double k_RobotRadius = .000000000001; //TODO Measure Radius

        public static final double k_wheelDiameter = 0.1016; //meters

        /*TODO for all of these change when robot is characterized*/
        public static final double k_MaxTeleSpeed = Units.feetToMeters(16.43); //m/s TODO find through characterization
        public static final double k_MaxAutoSpeed = Units.feetToMeters(16.43); //m/s
        public static final double k_MaxAcceleration = Units.feetToMeters(.0000000000001); //m/s/s
        
        public static final double kXYjoystickCoefficient = .5;
        public static final double kMaxAngularSpeed = .0000000000001;
        public static final double kRotCoefficient = .25;

        public static final double k_driveEncoderTicksPerRotation = 2048; 
        public static final double k_gearRatio = 6.75;

        //public static double fieldCalibration = 0;

        //Angle offsets
        public static double leftFrontOffset = .0000000000001;
        public static double rightFrontOffset = .0000000000001;
        public static double leftRearOffset = .0000000000001;
        public static double rightRearOffset = .0000000000001;
    
        public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(k_WheelBase / 2, k_TrackWidth / 2),    // Left Front
            new Translation2d(k_WheelBase / 2, -k_TrackWidth / 2),   // Right Front
            new Translation2d(-k_WheelBase / 2, k_TrackWidth / 2),   // Left Rear
            new Translation2d(-k_WheelBase / 2, -k_TrackWidth / 2)); // Right Rear

    }

    /**
     * The CAN Bus device IDs for devices used with the Swerve Drive: motor controllers and encoders
    */
    public static final class CANBusIDs
    { //TODO Change IDs
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
    }

    public static final class DriveModulePIDValues
    {
        //TODO Set actual PID values
        public static double k_driveP = 0.028; 
        public static double k_driveI = 0.0;
        public static double k_driveD = 0.025;
        public static double k_driveFF = 1/Units.feetToMeters(14.4);
        public static final int k_ToleranceInTicks = 15; //TODO Figure out real value
    }

    public static final class SteerModulePIDValues
    {
        //TODO Set actual PID values
        public static double k_steerP = 0.028; 
        public static double k_steerI = 0.0;
        public static double k_steerD = 0.025;
        public static double k_steerFF = 0.0;

        public static final double k_steerDeadband = 0.02; // Deadband on the motor controller
        public static final double k_ToleranceInDegrees = .1; //TODO Figure out real value
    }


}
