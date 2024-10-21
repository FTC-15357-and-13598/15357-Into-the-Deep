package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.robomossystem.*;

/**
 * This is a test class of a variety of functions, not to be used.
 */

public class robotTelemetry {
    // Telemetry Constructor
    public robotTelemetry(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Define a constructor that allows the OpMode to pass a reference
    private LinearOpMode myOpMode;

    //Classes that will be referenced need to be added
    private specimenElevator specimenElevator = new specimenElevator(myOpMode);
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(myOpMode);

    // FTC Dashboard - Access at 192.168.43.1:8080/dash - See packets later on in the code
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    /* Like periodic in subsystems, this will be called by teleop and auton programs
     * this void will populate the telemetry on the driver station and FTC dashboard
     * based on constants defined. The FTC dashboard is not allowed during competition
     * so most of the time telemetry will be enabled and FTC dashboard will not
     */

    public void periodic() {
        if (Constants.Telemetry.showTelemetry) {
            //Specimen elevator telemetry
            myOpMode.telemetry.addData("Specimen Elevator", specimenElevator.position);

            // Drive train telemetry


            //Update dashboard must be last
            myOpMode.telemetry.update();

        }
        if (Constants.Telemetry.showDashBoard){
            //specimen elevator parameters
            packet.put("Specimen Elevator Position",specimenElevator.position);
            packet.put("Specimen Elevator Target",specimenElevator.target);
            packet.put("Specimen Elevator Power",specimenElevator.power);

            //drivetrain parameters


            //send TelemetryPacket must be last
            dashboard.sendTelemetryPacket(packet);
        }



    }
}
