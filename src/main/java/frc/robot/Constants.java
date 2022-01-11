// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

        /*TODO for all of these change when robot is characterized*/
        public static final double k_MaxSpeed = .0000000000001;
        public static final double k_MaxAcceleration = .0000000000001;
        
        public static final double kXYjoystickCoefficient = .5;
        public static final double kMaxAngularSpeed = .0000000000001;
        public static final double kRotCoefficient = .25;

        public static double fieldCalibration = 0;

        //Angle offsets
        public static double frontLeftOffset = .0000000000001;
        public static double frontRightOffset = .0000000000001;
        public static double backLeftOffset = .0000000000001;
        public static double backRightOffset = .0000000000001;
    
    }

    /**
     * The CAN Bus device IDs for devices used with the Swerve Drive: motor controllers and encoders
    */
    public static final class CANBusIDs
    {

    }

    /**
    * The Motor controller we're using can control both brushed and brushless DC motors. 
    * We need to specify what kind of motor we're controlling.
    */
    public static final class MotorTypes
    {

    }


}
