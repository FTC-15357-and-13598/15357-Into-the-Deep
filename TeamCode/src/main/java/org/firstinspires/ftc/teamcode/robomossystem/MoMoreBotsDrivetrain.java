/* Created 9/12/2024 Brian Herioux
   Contiains Drivetrian code using SparkFun OTOS odemetry snesor
   Drivetrain cod used for MoBots and MoreBots
*/

package org.firstinspires.ftc.teamcode.robomossystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utility.Constants;

import java.util.List;

public class MoMoreBotsDrivetrain {
    // Public Members
    public double heading           = 0;
    public double otosXPostion;
    public double otosYPostion;
    public double otosHead;

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl2 driveController     = new ProportionalControl2(Constants.Drivetrain.DRIVE_GAIN, Constants.Drivetrain.DRIVE_ACCEL,
            Constants.Drivetrain.DRIVE_MAX_AUTO, Constants.Drivetrain.DRIVE_TOLERANCE, Constants.Drivetrain.DRIVE_DEADBAND, false);
    public ProportionalControl2 strafeController    = new ProportionalControl2(Constants.Drivetrain.STRAFE_GAIN, Constants.Drivetrain.STRAFE_ACCEL,
            Constants.Drivetrain.STRAFE_MAX_AUTO, Constants.Drivetrain.STRAFE_TOLERANCE, Constants.Drivetrain.STRAFE_DEADBAND, false);
    public ProportionalControl2 yawController       = new ProportionalControl2(Constants.Drivetrain.YAW_GAIN, Constants.Drivetrain.YAW_ACCEL,
            Constants.Drivetrain.YAW_MAX_AUTO, Constants.Drivetrain.YAW_TOLERANCE,Constants.Drivetrain.YAW_DEADBAND, true);

    //SparkfunOtos is myOtos
    SparkFunOTOS myOtos;
    private final int READ_PERIOD = 1;

    // ---  Private Members

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    /* // FTC Dashboard - Access at 192.168.43.1:8080/dash - See packets later on in the code
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();*/

    private LinearOpMode myOpMode;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private double previousOtosHeading = 0;
    private double previousTime = 0;

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private double otosTurn           = 0; // Latest Robot Turn Rate from OTOS
    private boolean showTelemetry     = true; // set to true to display telemetry

    // Robot Constructor
    public MoMoreBotsDrivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    private void configureOTOS() {
        myOtos.setLinearUnit(DistanceUnit.INCH); //Units are inches
        myOtos.setAngularUnit(AngleUnit.DEGREES); //And in degrees
        myOtos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0)); //This sets current position to 0,0,0
        myOtos.setLinearScalar(0.979); //This sets the linear scalar to 1.0, can define this later once robot is built and determine the scaling.
        myOtos.setAngularScalar(1.0); //This sets the angular scalar to 1.0, can define this later once robot is built and determine the scaling.
        myOtos.resetTracking(); //This resets the tracking of the sensor
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0)); //This sets the position of the sensor to 0,0,90 as the sensor is currently turned 90 degrees
        myOtos.calibrateImu(255, false); //Always calibrate the IMU
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry)
    {
        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        // !!! BadMonkey has a reversed motor. Hence the odd forward/reverse below
        leftFrontDrive  = setupDriveMotor(Constants.Drivetrain.MOTOR_LF, Constants.Drivetrain.LF_Direction);
        rightFrontDrive = setupDriveMotor(Constants.Drivetrain.MOTOR_RF, Constants.Drivetrain.RF_Direction);
        leftBackDrive  = setupDriveMotor(Constants.Drivetrain.MOTOR_LB, Constants.Drivetrain.LB_Direction);
        rightBackDrive = setupDriveMotor(Constants.Drivetrain.MOTOR_RB, Constants.Drivetrain.RB_Direction);
        imu = myOpMode.hardwareMap.get(IMU.class, Constants.Drivetrain.IMU);

        // Connect to the OTOS
        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class,Constants.Drivetrain.Otos);
        configureOTOS();

        //  Connect to the encoder channels using the name of that channel.
        //driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "axial");
        //strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "lateral");

        //Connect driveEncoder to the pos.y of myOtos encoder
        //double driveEncoder = myOtos.getPosition().y;
        //Connect strafeEncoder to the pos.x of myOtos encoder
        //double strafeEncoder = myOtos.getPosition().x;


        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        // We currently still use the REV IMU and not the OTOS Imu. Will need more testing to decide how to proceed.
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(Constants.Drivetrain.HUB_LOGO, Constants.Drivetrain.HUB_USB);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

       // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public void periodic () {
        double currentTime = myOpMode.getRuntime();
        otosXPostion = myOtos.getPosition().x;
        otosYPostion = myOtos.getPosition().y;
        otosHead = myOtos.getPosition().h;

        // Calculate angular velocity from OTOS heading
        double deltaTime = currentTime - previousTime;
        if (deltaTime > 0) {
            otosTurn = (otosHead - previousOtosHeading) / deltaTime;
        }

        // Update previous heading and time
        previousOtosHeading = otosHead;
        previousTime = currentTime;

        //Get IMU heading, used for FC teleop drive
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        heading  = orientation.getYaw(AngleUnit.DEGREES);
        turnRate    = angularVelocity.zRotationRate;

        // Big Telemetry block to show all the values. myOpMode is for the Driver Station and packet.put is for the Dashboard.

        if (Constants.Telemetry.showTelemetry) {myOpMode.telemetry.addData("OTOS Heading", "%5.2f", otosHead);
         myOpMode.telemetry.addData("OTOS X: Y:", "%5.2f %5.2f", otosXPostion, otosYPostion);
         myOpMode.telemetry.addData("OTOS Heading", "%5.2f", otosHead);
         myOpMode.telemetry.addData("OTOS TurnRate", "%5.2f", otosTurn);
         myOpMode.telemetry.addData("imu turn rate", turnRate);
         }



    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        /*
        resetOdometry();


        driveController.reset(distanceInches, power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();
        //myOtos.resetTracking(); // TODO check if this is necessary

        while (myOpMode.opModeIsActive()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        myOtos.resetTracking();
        stopRobot();
        */
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        /*
        resetOdometry();

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(distanceInches, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot(); */
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {
        // @TODO This function does not use the odometry wheels
        /*


        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
        myOtos.resetTracking(); */
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param xDistanceInches  Distance to travel.  +ve = fordward, -ve = reverse.
     * @param yDistanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */

    public void driveXY(double xDistanceInches, double yDistanceInches, double power, double holdTime) {
        /*
        resetOdometry(); // Reset odometry at the start of the move

        driveController.reset(xDistanceInches, power);   // Set desired drive distance
        strafeController.reset(yDistanceInches, power);  // Set desired strafe distance
        yawController.reset();                           // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive()) {
            // Implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot(); */
    }


    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        /*if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }*/
    }
    /**
     * Field Centric Drive to be called by teliop with inputs from driver's controller
     * @param leftX    Fwd/Rev axis power
     * @param leftY    Left/Right axis power
     * @param rightX       Yaw axis power
     * @param turbofactor   Speed factor for teliop 1= full speed make <1 to slow down
     */
    public void moveRobotFC(double leftX, double leftY, double rightX, double turbofactor){

        double botHeading =AngleUnit.normalizeRadians(heading);
        double rotX = leftX * Math.cos(botHeading) - leftY * Math.sin(botHeading);
        double rotY = leftX * Math.sin(botHeading) + leftY * Math.cos(botHeading);

        /*
         Denominator is the largest motor power (absolute value) or 1 and manipulated by
         the trubofactor to give the driver more control in close quarters.
         This ensures all the powers maintain the same ratio, but only when
         at least one is out of the range [-1, 1]
         insure 1.0<=turbofactor<=0.2 to not overdrive or go too slow in case of error
         in calling function. turbofactor is set to 1 if greater than 1, 0.2 if less
         than 0.2.
        */
        turbofactor = Range.clip(turbofactor,0.2,1.0);

        double denominator = Math.max(Math.abs(leftY) + Math.abs(leftX) + Math.abs(rightX), 1)/turbofactor;
        double lF = (rotY + rotX + rightX) / denominator;
        double lB = (rotY - rotX + rightX) / denominator;
        double rF = (rotY - rotX - rightX) / denominator;
        double rB = (rotY + rotX - rightX) / denominator;



        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        /*if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }*/
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Set odometry counts and distances to zero.
     */
    /*public void resetOdometry() {

        //myOtos.resetTracking; // TODO Moved from the end. 9/3/2024
        //myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveController.reset(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeController.reset(0);

    }*/


    //Neither of these are used it AUTON.
    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }

    public void resetIMUyaw() {imu.resetYaw();}
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class ProportionalControl2 {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl2(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {return setPoint;}

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }

}