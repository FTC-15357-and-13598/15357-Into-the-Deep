package org.firstinspires.ftc.teamcode.robomossystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utility.Constants;
import org.firstinspires.ftc.teamcode.opmode.BasicLinearFieldCent;

//Create class for subsystem
public class specimenElevator {
    // Elevator Constructor
    public specimenElevator(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Declare motor with encoder
    public static DcMotor myMotor = null;

    // Define a constructor that allows the OpMode to pass a reference
    private LinearOpMode myOpMode;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     **/

    public String specimenPosition = null;

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
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /** This void will be called by both Teleop and Auton to execute periodic items required when
     * other items are not being called by the teleop or auton. Variables declared between this
     * comment and the void are updated by the void for use by the teleop, auton or telemetry
     * class which sends items telemetry or the dashboard
    **/

    public int position, target;
    public double power;
    public boolean AtTarget;

    public void periodic(){
        position =myMotor.getCurrentPosition(); // For adding to dashboard
        target= myMotor.getTargetPosition();    // For adding to dashboard
        power= myMotor.getPower();              // For adding to dashboard
        AtTarget = (Math.abs(position-target)<Constants.Specimen.tolerance);    // For adding to dashboard
        // If motor is within tolerance set motor power to 0 enabling the break
        /*TODO: following if statement commented out to verify it is not causing the motor
        *  to no go down under power. Need to verify that motor is not overheating when down */
        if (!myMotor.isBusy() && (specimenPosition == "down")){
            myMotor.setPower(0.0);
        }
    }

    public void toDown() {
        // Move elevator to bottom
        myMotor.setTargetPosition(Constants.Specimen.DownPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        specimenPosition = "down";
    }

    public void lowBar() {
        // Move elevator to Low Bar
        myMotor.setTargetPosition(Constants.Specimen.LowBarPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        specimenPosition = "lowbar";
    }

    public void highBar() {
        // Move elevator to High Bar
        myMotor.setTargetPosition(Constants.Specimen.HighBarPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        specimenPosition = "highbar";
    }

    public void unHook() {
        // Raise to unhook specimen
        int newSP = Range.clip(myMotor.getTargetPosition()+Constants.Specimen.HookMove,Constants.Specimen.DownPosition,Constants.Specimen.HighBarPosition);
        myMotor.setTargetPosition(newSP);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
    }

    public void hook() {
        // Raise to unhook specimen
        int newSP = Range.clip(myMotor.getTargetPosition()-Constants.Specimen.HookMove,Constants.Specimen.DownPosition,Constants.Specimen.HighBarPosition);
        myMotor.setTargetPosition(newSP);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
    }
    public void hookAuto() {
        // Raise to unhook specimen
        int newSP = Range.clip(myMotor.getTargetPosition()-Constants.Specimen.HookMove,Constants.Specimen.DownPosition,Constants.Specimen.HighBarPosition);
        myMotor.setTargetPosition(newSP);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Specimen.defaultPower);
        while (myMotor.isBusy()){}
    }
}
