package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    // Create subclasses for the Drivetrain and each control item such as servos and motors.

    public static final class Drivetrain {
        // Hardware Configuration for Drivetrain Items
        // ------------------------------------------------------------

        // Drivetrain Motors, Define configured name and direction
        public static final String MOTOR_LF = "motorFrontLeft";
        public static final DcMotorSimple.Direction LF_Direction =DcMotorSimple.Direction.FORWARD;
        public static final String MOTOR_LB = "motorBackLeft";
        public static final DcMotorSimple.Direction LB_Direction =DcMotorSimple.Direction.REVERSE;
        public static final String MOTOR_RF = "motorFrontLeft";
        public static final DcMotorSimple.Direction RF_Direction =DcMotorSimple.Direction.FORWARD;
        public static final String MOTOR_RB = "motorBackRight";
        public static final DcMotorSimple.Direction RB_Direction =DcMotorSimple.Direction.FORWARD;

        //Define name of IMU on Control hub and Control lHub Orientation
        public static final String IMU = "imu";
        public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Define Devicename for Otos
        public static final String Otos ="sensor_otos";



    }







}