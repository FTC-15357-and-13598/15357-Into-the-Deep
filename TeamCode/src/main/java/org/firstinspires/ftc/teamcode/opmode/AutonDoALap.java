/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robomossystem.*;
import org.firstinspires.ftc.teamcode.utility.Constants;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Do a Lap", group = "Into the Deep")
public class AutonDoALap extends LinearOpMode
{
    // Get instance of Dashboard. Make sure update telemetry and sen packet are at end of opmode
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    boolean dummy;

    // get an instance of each subsystem class.
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(this);
    private bucketElevator bucketElevator = new bucketElevator(this);
    private specimenElevator specimenElevator = new specimenElevator(this);
    private intakeSubSystem intakeSubSystem = new intakeSubSystem(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        drivetrain.initialize(2);
        bucketElevator.init();
        specimenElevator.init();
        intakeSubSystem.init();

        int step = 1;

        waitForStart();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            /* Call periodic for subsystems that have a periodic voids,
             do these first as some of the voids called by the commands and other sequences
             require data from the periodic functions.
             */
            specimenElevator.periodic();
            bucketElevator.periodic();
            dummy =drivetrain.periodic(); //This is called as a variable for scheduling reasons
            intakeSubSystem.periodic();
            // We will be putting steps in here
/*
        switch (step){
            case 1 :
                specimenElevator.highBar();
                robot.gotoPosition(29.2,62,180,.25,1);
                step=step+1;

            case 2 :
                specimenElevator.unHook();
                while (specimenElevator.myMotor.isBusy()){}
                step=step+1;

            case 3 :
                robot.gotoPosition(5,62,0,.25,1);
                specimenElevator.toDown();
                step=step+1;
        }*/
            drivetrain.gotoPosition(0,24,90,.4,0);
            drivetrain.gotoPosition(0,96,90,.4,0);
            drivetrain.gotoPosition(72,96,180,.4,0);
            drivetrain.gotoPosition(96,96,-90,.4,0);
            drivetrain.gotoPosition(96,24,-90,.4,0);
            drivetrain.gotoPosition(96,0,0,.4,0);
            drivetrain.gotoPosition(0,0,0,.4,0);


            //update dashboard and telemetry if used
            if (Constants.Telemetry.showTelemetry) {
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
                packet.put("Specimen Elevator Position", specimenElevator.position);
                packet.put("Specimen Elevator Target", specimenElevator.target);
                packet.put("Specimen Elevator Power", specimenElevator.power);
                packet.put("Specimen Elevator at Target", specimenElevator.AtTarget);
                packet.put("Bucket Elevator Position", bucketElevator.position);
                packet.put("Bucket Elevator Target", bucketElevator.target);
                packet.put("Bucket Elevator at Target", bucketElevator.AtTarget);
                packet.put("Bucket Elevator Motor Power", bucketElevator.power);

                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}