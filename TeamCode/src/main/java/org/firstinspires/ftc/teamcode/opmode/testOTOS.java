/* Test Teleop to use to calibrate OTOS. To use place robot close to the wall with
the front facing out. dpad up will drive 100 inches, dpad right will turn 90 degrees
in the negative direction, dpad left will turn 90 degrees in the positive direction
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robomossystem.*;
import org.firstinspires.ftc.teamcode.utility.Constants;

@TeleOp(name="Test Otos", group = "Into the Deep")
public class testOTOS extends LinearOpMode
{
    // Get instance of Dashboard. Make sure update telemetry and sen packet are at end of opmode
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    boolean dummy;
    boolean done =false;

    // get an instance of each subsystem class.
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(this);
    private bucketElevator bucketElevator = new bucketElevator(this);
    private specimenElevator specimenElevator = new specimenElevator(this);
    private intakeSubSystem intakeSubSystem = new intakeSubSystem(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        drivetrain.initialize(2);
        bucketElevator.init(true);
        specimenElevator.init(true);
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
            if (gamepad1.dpad_up){
                done = false;
                drivetrain.gotoPosition(drivetrain.otosXPostion,(drivetrain.otosYPostion+100),drivetrain.otosHead,.2,5);
                done = true;
            }

            if (gamepad1.dpad_down){
                done =false;
                drivetrain.gotoPosition(drivetrain.otosXPostion,(drivetrain.otosYPostion-100),drivetrain.otosHead,.2,5);
                done =true;
            }

            if (gamepad1.dpad_right){
                done =false;
                drivetrain.gotoPosition(drivetrain.otosXPostion,drivetrain.otosYPostion,(drivetrain.otosHead-90),.2,5);
                done =true;
            }

            if (gamepad1.dpad_left){
                done =false;
                drivetrain.gotoPosition(drivetrain.otosXPostion,drivetrain.otosYPostion,(drivetrain.otosHead+90),.2,5);
                done =true;
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
}