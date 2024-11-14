package org.firstinspires.ftc.teamcode.robomossystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Constants;

/**
 * This is a test class of a variety of functions, not to be used.
 */
// TODO Turn off motor when not in use
public class bucketElevator {
    // Elevator Constructor
    public bucketElevator(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Declare motor with encoder and servo
    private static DcMotor myMotor = null;
    private static Servo myServo = null;

    // Define a constructor that allows the OpMode to pass a reference
    private LinearOpMode myOpMode;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <
     * All of the hardware devices are accessed via the hardware map, and initialized.
     **/
    public void init() {
        // Define and Initialize Motors and servos (note: need to use reference to actual OpMode).
        myMotor = myOpMode.hardwareMap.get(DcMotor.class, Constants.Bucket.MOTOR);
        myServo =myOpMode.hardwareMap.get(Servo.class, Constants.Bucket.Servo);
        myServo.setPosition(0.4);

        // Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // Set to FORWARD if using AndyMark motors
        myMotor.setDirection(Constants.Bucket.Direction);
        
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
    public String bucketPosition = null;

    public void periodic(){
        position =myMotor.getCurrentPosition(); // For adding to dashboard
        target= myMotor.getTargetPosition();    // For adding to dashboard
        power= myMotor.getPower();              // For adding to dashboard
        AtTarget = (Math.abs(position-target)<Constants.Bucket.tolerance);  // For adding to dashboard
        // If motor is within tolerance set motor power to 0 enabling the break
        /*TODO: check motor to insure it is not overheating. If statement below was commented
        *  out to insure it is not causing the elevator to not go down correctly. */
        if (!myMotor.isBusy() && (bucketPosition == "down")){
            myMotor.setPower(0.0);
        }
    }

    public void toDown() {
        // Move elevator to bottom
        myMotor.setTargetPosition(Constants.Bucket.DownPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Bucket.defaultPower);
        bucketPosition = "down";
    }

    public void lowBucket() {
        // Move elevator to Low Bar
        myMotor.setTargetPosition(Constants.Bucket.LowBucketPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Bucket.defaultPower);
        bucketPosition = "lowBucket";
    }

    public void highBucket() {
        // Move elevator to High Bar
        myMotor.setTargetPosition(Constants.Bucket.HighBucketPosition);
        myMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myMotor.setPower(Constants.Bucket.defaultPower);
        bucketPosition = "highBucket";
    }

    public void servoDump (){
        myServo.setPosition(Constants.Bucket.dumpPosition);
            myOpMode.telemetry.addData("Servo Position",Constants.Bucket.dumpPosition);
    }
    public void servoRecieve (){
        myServo.setPosition(Constants.Bucket.recievePosition);
        myOpMode.telemetry.addData("Servo Position",Constants.Bucket.recievePosition);
    }
    public int step=0;
    public void scoreBucket() {
        step=1;
    }

}
