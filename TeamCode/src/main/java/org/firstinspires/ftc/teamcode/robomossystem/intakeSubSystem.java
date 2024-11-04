package org.firstinspires.ftc.teamcode.robomossystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.Constants;

/**
 * This is a test class of a variety of functions, not to be used.
 */
// TODO Turn off motor when not in use
public class intakeSubSystem {
    // Intake Constructor
    public intakeSubSystem(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Declare motor with encoder and servo
    private static DcMotor myMotor = null;
    private static Servo slideServo1 = null;
    private static Servo doorServo2 = null;
    private static Servo armRotationServo3 = null;
    private static Servo intakeServo = null;

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
        myMotor = myOpMode.hardwareMap.get(DcMotor.class, Constants.INTAKE.MOTOR);
        slideServo1 = myOpMode.hardwareMap.get(Servo.class, Constants.INTAKE.slideServo);
        doorServo2 = myOpMode.hardwareMap.get(Servo.class, Constants.INTAKE.doorServo);
        armRotationServo3 = myOpMode.hardwareMap.get(Servo.class, Constants.INTAKE.armRotationServo);
        intakeServo = myOpMode.hardwareMap.get(Servo.class, Constants.INTAKE.intakeServo);

        // Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // Set to FORWARD if using AndyMark motors
        myMotor.setDirection(Constants.INTAKE.Direction);

        // Set the drive motor modes to run with and 0 encoder
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    /** This void will be called by both Teleop and Auton to execute periodic items required when
     * other items are not being called by the teleop or auton. Variables declared between this
     * comment and the void are updated by the void for use by the teleop, auton or telemetry
     * class which sends items telemetry or the dashboard
     **/

    public int position, target;
    public double power;
    public String armPosition;

    public void periodic(){
        //position =myMotor.getCurrentPosition();
        //target= myMotor.getTargetPosition();
        power= myMotor.getPower();
        // If motor is within tolerance set motor power to 0 enabling the break
        if (AtTarget && (target == Constants.Bucket.DownPosition)){
            myMotor.setPower(0.0);
        }
    }

    public boolean AtTarget = (Math.abs(position-target)<Constants.Bucket.tolerance);

    public void intakeForward() {
        // Run the intake motor in the forwrd direction
        //myMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //myMotor.setPower(Constants.INTAKE.defaultPower);
        //while (myMotor.isBusy()) {}
        intakeServo.setPosition(1.0);

    }

    public void intakeReverse() {
        // reverse the Intake motor in case need to spit part out
        //myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //myMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //myMotor.setPower(Constants.INTAKE.defaultPower);
        //while (myMotor.isBusy()) {}
        intakeServo.setPosition(0.0);
    }

    public void intakeStop(){
        intakeServo.setPosition(0.5);
    }

    public void intakeSlideForward (){
        slideServo1.setPosition(Constants.INTAKE.forwardPosition);
        myOpMode.telemetry.addData("Servo Position",Constants.INTAKE.forwardPosition);
    }
    public void intakeSlideReverse (){
        slideServo1.setPosition(Constants.INTAKE.backPosition);
        myOpMode.telemetry.addData("Servo Position",Constants.INTAKE.backPosition);
    }

    public void armDownPosition(){
        armRotationServo3.setPosition(Constants.INTAKE.armDownPosition);
        myOpMode.telemetry.addData("Servo Position",Constants.INTAKE.armDownPosition);
        armPosition = "down";
    }
    public void armMidPosition(){
        armRotationServo3.setPosition(Constants.INTAKE.armMidPosition);
        myOpMode.telemetry.addData("Servo Position",Constants.INTAKE.armMidPosition);
        armPosition = "middle";
    }

    public void armUpPosition(){
        armRotationServo3.setPosition(Constants.INTAKE.armUpPosition);
        myOpMode.telemetry.addData("Servo Position",Constants.INTAKE.armUpPosition);
        armPosition = "up";
    }

    public void doorPositionClosed(){
        doorServo2.setPosition(Constants.INTAKE.doorClosePosition);
        myOpMode.telemetry.addData("Servo Position", Constants.INTAKE.doorClosePosition);
    }

    public void doorPositionOpen(){
        doorServo2.setPosition(Constants.INTAKE.doorOpenPosition);
        myOpMode.telemetry.addData("Servo Position", Constants.INTAKE.doorOpenPosition);
    }

    //Void and variables to sequence transferring sample from intake to bucket
    public String tranfering = "no"; //Use used to indicate transfer in progress
    public int step =0; //Used to counts steps in sequence set back to 0 when done
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.
    boolean case1stRun = false;

    public void startTransfer() {
        tranfering="yes";
        step=1;
    }

    public void transfer2bucket(){
        switch (step){
            case 1: //retract slide, move arm to mid
                if (!case1stRun){ // only reset holdtimer on first run of case
                    holdTimer.reset();
                    case1stRun = true;
                }
                armMidPosition();
                intakeSlideReverse();
                if (holdTimer.time() > 1) {
                    step=2;  //Move on to next step
                    case1stRun = false;
                }
            case 2: //Move arm up
                armUpPosition();
                if (!case1stRun){ // only reset holdtimer on first run of case
                    holdTimer.reset();
                    case1stRun = true;
                }
                if (holdTimer.time() > 1) {
                    step=3;  //Move on to next step
                    case1stRun = false;
                }
            case 3:  // open door
                doorPositionOpen();
                if (!case1stRun){ // only reset holdtimer on first run of case
                 holdTimer.reset();
                 case1stRun = true;
                }
                if (holdTimer.time() > 1) {
                    step=4;  //Move on to next step
                    case1stRun = false;
                }
            case 4: //run intake to spit out
                intakeForward();
                if (!case1stRun){ // only reset holdtimer on first run of case
                    holdTimer.reset();
                    case1stRun = true;
                }
                if (holdTimer.time() > 2) {
                    intakeStop();
                    armMidPosition();
                    doorPositionClosed();
                    step=0;  //End sequence
                    case1stRun =false;
                }
        }

    }
}
