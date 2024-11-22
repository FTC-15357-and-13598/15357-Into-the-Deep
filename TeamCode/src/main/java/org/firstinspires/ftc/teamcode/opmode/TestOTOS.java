
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robomossystem.MoMoreBotsDrivetrain;
import org.firstinspires.ftc.teamcode.robomossystem.bucketElevator;
import org.firstinspires.ftc.teamcode.robomossystem.intakeSubSystem;
import org.firstinspires.ftc.teamcode.robomossystem.specimenElevator;
import org.firstinspires.ftc.teamcode.utility.Constants;

@TeleOp(name="Test OTOS", group="Linear OpMode")


public class TestOTOS extends LinearOpMode {
    //get an instances of subsystem classes.
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(this);
    private specimenElevator specimenElevator = new specimenElevator(this);
    private bucketElevator bucketElevator = new bucketElevator(this);
    private intakeSubSystem intakeSubSystem = new intakeSubSystem(this);

    // Get instance of Dashboard. Make sure update telemetry and sen packet are at end of opmode
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    public String specimenPosition = null;
    boolean dummy;
    boolean done =true;
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
        drivetrain.initialize(2);
        specimenElevator.init(false);
        bucketElevator.init(false);
        intakeSubSystem.init();
        //specimenElevator.toDown();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Initialize the robot hardware & Turn on telemetry
            drivetrain.initialize(2);
            bucketElevator.init(true);
            specimenElevator.init(true);
            intakeSubSystem.init();

            int step = 1;

            waitForStart();

            // Run Auto if stop was not pressed.
            if (opModeIsActive()) {
            /* Call periodic for subsystems that have a periodic voids,
             do these first as some of the voids called by the commands and other sequences
             require data from the periodic functions.
             */
                specimenElevator.periodic();
                bucketElevator.periodic();
                dummy = drivetrain.periodic(); //This is called as a variable for scheduling reasons
                intakeSubSystem.periodic();
                // We will be putting steps in here
                if (gamepad1.dpad_up) {
                    done = false;
                    drivetrain.gotoPosition((drivetrain.otosXPostion+100), drivetrain.otosYPostion, drivetrain.otosHead, .2, 5);
                    done = true;
                }

                if (gamepad1.dpad_down) {
                    done = false;
                    drivetrain.gotoPosition((drivetrain.otosXPostion-100),drivetrain.otosYPostion, drivetrain.otosHead, .2, 5);
                    done = true;
                }

                if (gamepad1.dpad_right) {
                    done = false;
                    drivetrain.gotoPosition(drivetrain.otosXPostion, drivetrain.otosYPostion, (drivetrain.otosHead - 90), .2, 5);
                    done = true;
                }

                if (gamepad1.dpad_left) {
                    done = false;
                    drivetrain.gotoPosition(drivetrain.otosXPostion, drivetrain.otosYPostion, (drivetrain.otosHead + 90), .2, 5);
                    done = true;
                }


                //update dashboard and telemetry if used
                if (Constants.Telemetry.showTelemetry) {
                    telemetry.addData("Move done:", done);
                    telemetry.update();
                }
                if (Constants.Telemetry.showDashBoard) {
                    packet.put("OTOS Heading", drivetrain.otosHead);
                    packet.put("OTOS X", drivetrain.otosXPostion);
                    packet.put("OTOS Y", drivetrain.otosYPostion);
                    packet.put("Bot Heading", drivetrain.heading);
                    packet.put("Left Front Power", drivetrain.LFpower);
                    packet.put("Left Rear Power", drivetrain.LRpower);
                    packet.put("Right Front Power", drivetrain.RFpower);
                    packet.put("Right Rear Power", drivetrain.RRpower);

                    dashboard.sendTelemetryPacket(packet);
                }
            }
        }
        //Put any super-system type voids here
        //TODO add timers in place of sleep

    }
}

