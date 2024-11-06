package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Constants {
    // Create subclasses for the Drivetrain and each control item such as servos and motors.

    public static final class Drivetrain {
        // Hardware Configuration for Drivetrain Items
        // ------------------------------------------------------------

        // Drivetrain Motors, Define configured name and direction
        public static final String MOTOR_LF = "leftFrontDrive";
        public static final DcMotorSimple.Direction LF_Direction =DcMotorSimple.Direction.FORWARD;
        public static final String MOTOR_LB = "leftBackDrive";
        public static final DcMotorSimple.Direction LB_Direction =DcMotorSimple.Direction.FORWARD;
        public static final String MOTOR_RF = "rightFrontDrive";
        public static final DcMotorSimple.Direction RF_Direction =DcMotorSimple.Direction.FORWARD;
        public static final String MOTOR_RB = "rightBackDrive";
        public static final DcMotorSimple.Direction RB_Direction =DcMotorSimple.Direction.FORWARD;

        //Define name of IMU on Control hub and Control lHub Orientation Logo facing direction need to be revered depending on
        //if robot starts facing the wall or not
        public static final String IMU = "imu";
        public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_WALL = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_AWAY = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        // Position Tuning constants
        // TODO Tune gains and accels for robot. Currnently moves in an odd rhomboid way.

        public static final double X_GAIN          = 0.03;    // Strength of axial position control
        public static final double X_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
        public static final double X_TOLERANCE     = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
        public static final double X_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
        public static final double X_MAX_AUTO      = 0.9;     // "default" Maximum Axial power limit during autonomous

        public static final double Y_GAIN         = 0.03;    // Strength of lateral position control
        public static final double Y_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
        public static final double Y_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
        public static final double Y_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
        public static final double Y_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

        public static final double HEADING_GAIN            = 0.018;    // Strength of Yaw position control
        public static final double HEADING_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
        public static final double HEADING_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
        public static final double HEADING_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
        public static final double HEADING_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

        //Constants for OTOS
        public static final String Otos ="otos"; // Define Devicename for Otos

        //Todo define starting linear and angular scalers, offset on robot and starting pose
        public static final SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0,0); // Offset for Otos mounting on robot
        public static final double linearScaler = 0.979;
        public static final double angularScaler = 1.0;

        public static final SparkFunOTOS.Pose2D blueSpecimen = new SparkFunOTOS.Pose2D(0.0,0.0,0.0); // Starting position for blue specimen position
        public static final SparkFunOTOS.Pose2D blueBucket = new SparkFunOTOS.Pose2D(0.0,0.0,0.0); // Starting position for blue bucket position
        public static final SparkFunOTOS.Pose2D redSpecimen = new SparkFunOTOS.Pose2D(0.0,0.0,0.0); // Starting position for red specimen position
        public static final SparkFunOTOS.Pose2D redBucket = new SparkFunOTOS.Pose2D(0.0,0.0,0.0); // Starting position for red bucket position

    }
    public static final class Specimen {
        // Drivetrain Motors, Define configured name and direction
        public static final String MOTOR = "viperLeft";
        public static final DcMotorSimple.Direction Direction =DcMotorSimple.Direction.REVERSE;

        public static final double defaultPower         =0.5;
        public static final int tolerance               =2;

        public static final int DownPosition            = 10;
        public static final int LowBarPosition          = 213;
        public static final int HighBarPosition         = 1325;
        public static final int HookMove                = 50;
    }
    public static final class Bucket {
        // Drivetrain Motors, Define configured name and direction
        public static final String MOTOR = "viperRight";
        public static final DcMotorSimple.Direction Direction =DcMotorSimple.Direction.REVERSE;

        public static final double defaultPower         =0.5;
        public static final int tolerance               =2;

        public static final int DownPosition            = 5;
        public static final int LowBucketPosition       = 413;
        public static final int HighBucketPosition      = 3050; //1504->1800->2400>3200

        public static final String Servo                ="dumpServo";
        public static final double recievePosition      = 0.1;
        public static final double dumpPosition         = 1.0;
    }

    public static final class INTAKE {
        // Drivetrain Motors, Define configured name and direction
        public static final String MOTOR = "intakeMotor";
        public static final DcMotorSimple.Direction Direction =DcMotorSimple.Direction.REVERSE;

        public static final double defaultPower         =0.9;
        public static final int tolerance               =2;

        public static final int intakePosition            = 5;
        public static final int unLoadPosition       = 413;
        public static final int travelPosition      = 1504;

        public static final String intakeServo = "intakeServo";

        public static final String slideServo                ="intakeSlideServo";
        public static final double forwardPosition      = 0.1;
        public static final double backPosition         = 0.5;

        public static final String doorServo            ="intakeDoor";
        public static final double doorOpenPosition         =0.5;
        public static final double doorClosePosition        =0.0;

        public static final String armRotationServo          ="armRotationServo";
        public static final double armDownPosition       =0.5;
        public static final double armMidPosition        =0.25;
        public static final double armUpPosition         =0.0;

    }

    public static final class Telemetry {
        public static final boolean showTelemetry       =true;
        public static final boolean showDashBoard       =true;
    }







}