package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robomossystem.MoMoreBotsDrivetrain;
import org.firstinspires.ftc.teamcode.robomossystem.bucketElevator;
import org.firstinspires.ftc.teamcode.robomossystem.intakeSubSystem;
import org.firstinspires.ftc.teamcode.robomossystem.specimenElevator;
import org.firstinspires.ftc.teamcode.utility.Constants;

@Autonomous(name="Score Bucket", group = "Into the Deep",preselectTeleOp = "Field Centered Teleop")
public class AutonBucket extends LinearOpMode
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

        switch (step){
            case 1 :
                intakeSubSystem.armMidPosition();
                sleep(500);
                bucketElevator.highBucket();
               // drivetrain.gotoPosition(13,129.5,0,.2,1);
                drivetrain.gotoPosition(13,129.5,-45,.25,.5);

                bucketElevator.servoDump();
                sleep(850);
                bucketElevator.servoRecieve();
                bucketElevator.toDown();
                step=step+1;
            case 2 :
                step=step+1;
                //drivetrain.gotoPosition(20,127,-45,.2,0);
               // drivetrain.gotoPosition(20,127,0,.2,0);
                //drivetrain.gotoPosition(26,128,0,.2,0);
                drivetrain.gotoPosition(26,129,-31,.3,.5);
                intakeSubSystem.armDownPosition();
                sleep(250);
                intakeSubSystem.intakeReverse();
                intakeSubSystem.intakeSlideForward();
                drivetrain.gotoPosition(37,121,-31,.3,0);
                intakeSubSystem.intakeStop();
                intakeSubSystem.armUpPosition();
                intakeSubSystem.intakeSlideReverse();
                sleep(250);
                intakeSubSystem.intakeReverse();
                drivetrain.gotoPosition(24,129,0,.3,.15);

            case 3 :
                intakeSubSystem.armMidPosition();
                intakeSubSystem.intakeStop();
                sleep(250);
                bucketElevator.highBucket();
                //drivetrain.gotoPosition(15,129.5,0,.2,0);
                drivetrain.gotoPosition(13,129.5,-45,.25,.5);
                bucketElevator.servoDump();
                sleep(850);
                bucketElevator.servoRecieve();
                bucketElevator.toDown();
                step=step+1;

            case 4:
                drivetrain.gotoPosition(23,128,0,.3,0.2);
                intakeSubSystem.intakeSlideOutAndArmDown();
                intakeSubSystem.intakeReverse();
                sleep(250);
                drivetrain.gotoPosition(32,128,0,.3,0);
                intakeSubSystem.intakeStop();
                intakeSubSystem.armUpPosition();
                intakeSubSystem.intakeSlideReverse();
                sleep(250);
                intakeSubSystem.intakeReverse();
                drivetrain.gotoPosition(24,129,0,.3,.15);
                intakeSubSystem.armMidPosition();
                intakeSubSystem.intakeStop();
                sleep(250);
                bucketElevator.highBucket();
                //drivetrain.gotoPosition(15,129.5,0,.2,0);
                drivetrain.gotoPosition(13,129.5,-45,.25,.5);
                bucketElevator.servoDump();
                sleep(850);
                bucketElevator.servoRecieve();
                bucketElevator.toDown();
                drivetrain.gotoPosition(29,127,35,.3,0.1);
                intakeSubSystem.intakeSlideOutAndArmDown();
                intakeSubSystem.intakeReverse();
                sleep(750);
                //drivetrain.gotoPosition(31,128,0,.25,0);
                intakeSubSystem.intakeStop();
                intakeSubSystem.armUpPosition();
                intakeSubSystem.intakeSlideReverse();
                sleep(250);
                intakeSubSystem.intakeReverse();
                drivetrain.gotoPosition(24,129,0,.3,.15);
                intakeSubSystem.armMidPosition();
                intakeSubSystem.intakeStop();
                bucketElevator.highBucket();
                drivetrain.gotoPosition(13,129.5,-45,.25,.5);
                bucketElevator.servoDump();
                sleep(850);
                bucketElevator.servoRecieve();
                bucketElevator.LowBar();

                drivetrain.gotoPosition(61,115,90,.55,0);
                drivetrain.gotoPosition(61.5,94,90,.55,0);
                bucketElevator.servoDump();
                sleep(2000);
        }
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