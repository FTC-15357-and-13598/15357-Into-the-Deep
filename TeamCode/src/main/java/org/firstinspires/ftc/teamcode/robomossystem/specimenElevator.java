package org.firstinspires.ftc.teamcode.robomossystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.Constants;

/**
 * This is a test class of a variety of functions, not to be used.
 */

public class specimenElevator {
    // Elevator Constructor
    public specimenElevator(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Declare two motors with encoders and two servos for the robot
    private static DcMotor myMotor = null;

    // Define a constructor that allows the OpMode to pass a reference
    private LinearOpMode myOpMode;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     **/
    public void init() {
        // Define and Initialize Motors and servos (note: need to use reference to actual OpMode).
        myMotor = myOpMode.hardwareMap.get(DcMotor.class, Constants.Specimen.MOTOR);

        // Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // Set to FORWARD if using AndyMark motors
        myMotor.setDirection(Constants.Specimen.Direction);
        
        // Set the drive motor modes to run with and 0 encoder
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void toDown() {
        // Move elevator to bottom
        myMotor.setTargetPosition(Constants.Specimen.DownPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        while (myMotor.isBusy()) {}
    }

    public void lowBar() {
        // Move elevator to Low Bar
        myMotor.setTargetPosition(Constants.Specimen.LowBarPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        while (myMotor.isBusy()) {}
    }

    public void highBar() {
        // Move elevator to High Bar
        myMotor.setTargetPosition(Constants.Specimen.HighBarPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        while (myMotor.isBusy()) {}
    }

    public void unHook() {
        // Raise to unhook specimen
        int newSP = myMotor.getTargetPosition()+Constants.Specimen.HookMove;
        myMotor.setTargetPosition(newSP);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        while (myMotor.isBusy()) {}
    }

    public void hook() {
        // Raise to unhook specimen
        int newSP = myMotor.getTargetPosition()-Constants.Specimen.HookMove;
        myMotor.setTargetPosition(newSP);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        while (myMotor.isBusy()) {}
    }
}
