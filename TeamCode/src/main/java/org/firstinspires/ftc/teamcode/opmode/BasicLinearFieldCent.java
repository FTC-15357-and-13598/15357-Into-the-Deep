
/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    * This file contains the MoBots / MoreBots Field Centric Linear "OpMode". This is the base
    * template for both robots. Built in functionality includes: field centric motion, via
    * MoreMoBOts drivetrain creating the bones for a full Tele Op.
    *
    * Field Centric is a method of driving a robot where the robot moves in the direction of the
    * joystick regardless of the robot's orientation. This is done by rotating the joystick input
    * by the robot's heading.
    *
    * This is READ ONLY. If you want to make your own, copy this class and paste it with a new name.
    *
    *
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.robomossystem.*;
import org.firstinspires.ftc.teamcode.utility.*;

@TeleOp(name="MOBots Core Basic Field Centered Template", group="Linear OpMode")


public class BasicLinearFieldCent extends LinearOpMode {
    //get an instance of the "drivetrain" class.
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(this);

    // get specimen elevator
    private specimenElevator specimenElevator = new specimenElevator(this);

    private bucketElevator bucketElevator = new bucketElevator(this);

    private intakeSubSystem intakeSubSystem = new intakeSubSystem(this);

    // Get instance of Dashboard. Make sure update telemetry and sen packet are at end of opmode
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    //private Servo servoTest = null;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         * These names are critical, label the front of the robot as FRONT. This will be
         * important later! Don't Iinitialize motors or IMU, they are part of the drivedrain.
         */

        /*
        * This initializes the servoTest servo. You would initialize other servos using the same method.
         */

        //servoTest = hardwareMap.get(Servo.class, "servoTest");

        //Initialize Drivetrain
        drivetrain.initialize(true);
        specimenElevator.init();
        bucketElevator.init();
        intakeSubSystem.init();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //If gamepad1 a is pressed move servotest to 0.05
            //Servo is a 5 rotation servo so 1 rotation is 360 degrees and 0.05 is 18 degrees
          //  if (gamepad1.a) {specimenElevator.();
           // }

            //If gamepad1 b is pressed move specimen to lowbar
            if (gamepad1.x) {specimenElevator.lowBar();
            }

            //If gamepad1 x move specimen elavator to high
            if (gamepad1.y) {specimenElevator.highBar();}

            //If gamepad1 y move specimen elevator to bottom
            if (gamepad1.a) {specimenElevator.toDown();}
            if (gamepad1.dpad_up) {specimenElevator.unHook();}
            if (gamepad1.dpad_down) {specimenElevator.hook();}
            //Bucket Elevator Commands
            if (gamepad2.dpad_up ) {bucketElevator.highBucket();}
            if (gamepad2.dpad_left) {bucketElevator.lowBucket();}
            if (gamepad2.dpad_down) {bucketElevator.toDown();}

            if (gamepad2.a) {bucketElevator.servoRecieve();}
            if (gamepad2.y) {bucketElevator.servoDump();}
            //Intake Slide Position
            if (gamepad2.x) {intakeSubSystem.intakeSlideForward();}
            if (gamepad2.b) {intakeSubSystem.intakeSlideReverse();}
            //Intake Motor Forward
            if (gamepad1.right_bumper)
                {intakeSubSystem.intakeForward();}
            else if (gamepad1.left_bumper)
                {intakeSubSystem.intakeReverse();}
            else
                {intakeSubSystem.intakeStop();}
            // Intake Arm Position
            if (gamepad2.left_bumper) {intakeSubSystem.armUpPosition();}
            if (gamepad2.right_bumper) {intakeSubSystem.armDownPosition();}
            //if (gamepad2.dpad_up) {intakeSubSystem.armMidPosition();}
            //Intake Door Position
            if (gamepad2.left_stick_button){intakeSubSystem.doorPositionClosed();}
            if (gamepad2.right_stick_button){intakeSubSystem.doorPositionOpen();}




            //Reset Yaw of IMU for FC drive if Driver hits back
            if (gamepad1.start) {drivetrain.resetIMUyaw();}

            /* Call Field Centric drive from drive train after calculating the speed factor
            the speed factor will be the fraction of full speed that full stick will result
            in. For example 1 is full speed, 0.5 is half speed. THe intention is to use the
            right trigger to to create a "Turbo" mode while allowing the driver to release
            the trigger and slow the robot down giving more control for small moves. a
             */
            double speedfact = 0.4;
            //If trigger pulled set speed factor to higher value
            if (gamepad1.right_trigger>0.1){speedfact =0.8;}
            //Call Field Centric void in drivetrain.
            drivetrain.moveRobotFC(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x,speedfact);

            // Call periodic for subsystems that have a periodic void
            specimenElevator.periodic();
            bucketElevator.periodic();
            drivetrain.periodic();
            intakeSubSystem.periodic();

            //update dashboard and telemetry if used
            if (Constants.Telemetry.showTelemetry) {telemetry.update();}
            if (Constants.Telemetry.showDashBoard) {
                packet.put("OTOS Heading",drivetrain.otosHead);
                packet.put("OTOS X",drivetrain.otosXPostion);
                packet.put("OTOS Y",drivetrain.otosYPostion);
                dashboard.sendTelemetryPacket(packet);}

        }
     }
    }


