/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;
import java.util.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name="39PointRedSide", group ="Concept")
public class test extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    // Hardware Variables
    private DcMotor rMotor;
    private DcMotor r2Motor;
    private DcMotor lMotor;
    private DcMotor l2Motor;
    private DcMotor liftlMotor;
    private DcMotor liftrMotor;
    private DcMotor inMotor;
    private Servo clawl;
    private Servo clawr;
    private DcMotor extender;
    private Servo grabber1;
    private Servo grabber2;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    // Constants for Encoder Drive
    private final double wheelDiameter = 4.5; //inches
    private final double encoderTicksfront = 228;
    private final double encoderTicksback = 1120;
    private final double circ = wheelDiameter * Math.PI; //wheel circumference
    private final double iptfront = circ / encoderTicksfront; // inches per tick
    private final double iptback =0.0179870166964285714285;
    private final double COUNTS_PER_INCH_FRONT = encoderTicksfront / circ;
    private final double COUNTS_PER_INCH_BACK = encoderTicksback / circ;
    private static final double DRIVE_SPEED = 0.3;     // Nominal speed for better accuracy.
    private static final double TURN_SPEED = 1;     // Nominal half speed for better accuracy.
    private static final double STRAFE_SPEED = 0.8;

    private double inchPerTickStrafe = 9.0 / 428.027222531353;
    private final double COUNT_PER_INCH_STRAFE = 1.0 / inchPerTickStrafe;

    // IMU Setup
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double power = .30;
    private double correction;
    PIDController pidRotate;
    PIDController pidDrive;
    double rotation;
    private static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF = 0.25;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZcA0xz/////AAABmarcn44zA0WEjW7lkswWRhNOWIJqsPTS9NMPr/8ipWUzygm+QM+aivRreupNzgYOzxB1bRB8hQhXvz1GWAeJJcFac5YM0JoqB3/ZSj8fxyupWne6gBxHk0MT0O4fVOQP4Baek4Y0qqeADOvhnhxD5zHN8HNUrQCOX+5Cq+CchK8VzKjJSpwIXzI/ukB/YBtuThHgVbhNR6b6ke5Uvt095mrlRThyMiTCrPkjorYe4AIZTGCjhVja859qeVUQk5Dfe/PKI5XtIr25Wk7sCXSS9DTVRgC0+Cr2MpB40rhLgLfdwvc3ViOPLBKbJSdMEn2Hgz/DidDirWkFsGacruDTcRorqORE55C0bYREWF3VEREc";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;

    @Override
    public void runOpMode() {
        inMotor = hardwareMap.get(DcMotor.class, "inMotor");
        rMotor = hardwareMap.get(DcMotor.class, "rMotor");
        r2Motor = hardwareMap.get(DcMotor.class, "r2Motor");
        lMotor = hardwareMap.get(DcMotor.class, "lMotor");
        l2Motor = hardwareMap.get(DcMotor.class, "l2Motor");
        liftlMotor = hardwareMap.get(DcMotor.class, "liftlMotor");
        liftrMotor = hardwareMap.get(DcMotor.class, "liftrMotor");
        clawl = hardwareMap.get(Servo.class, "clawl");
        clawr = hardwareMap.get(Servo.class, "clawr");
        extender = hardwareMap.get(DcMotor.class, "extender");
        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        inMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        pidRotate = new PIDController(.02, .003, 0);
        pidDrive = new PIDController(.05, 0, 0);
        initialize();
        IMUInit();
        setDirection();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        waitForStart();

    }




    public void reset() {
        r2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void power(double power) {
        r2Motor.setPower(power);
        l2Motor.setPower(power);
        rMotor.setPower(power);
        lMotor.setPower(power);
    }

    public void setNegativeDirection() {
        l2Motor.setDirection(DcMotor.Direction.FORWARD);
        lMotor.setDirection(DcMotor.Direction.FORWARD);
        r2Motor.setDirection(DcMotor.Direction.REVERSE);
        rMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDirection() {
        l2Motor.setDirection(DcMotor.Direction.REVERSE);
        lMotor.setDirection(DcMotor.Direction.REVERSE);
        r2Motor.setDirection(DcMotor.Direction.REVERSE);
        rMotor.setDirection(DcMotor.Direction.FORWARD);
        r2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initialize() {

        l2Motor = hardwareMap.get(DcMotor.class, "l2Motor");
        lMotor = hardwareMap.get(DcMotor.class, "lMotor");
        r2Motor = hardwareMap.get(DcMotor.class, "r2Motor");
        rMotor = hardwareMap.get(DcMotor.class, "rMotor");
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int pos;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        reset();
        lMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Determine new target position, and pass to motor controller
        pos = (rMotor.getCurrentPosition() + (int) (distance / iptback));
        r2Motor.setTargetPosition(pos);
        l2Motor.setTargetPosition(pos);
        rMotor.setTargetPosition(-pos);
        lMotor.setTargetPosition(pos);


        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        power(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (opModeIsActive() && (r2Motor.getCurrentPosition() <= (int) pos - 4 && rMotor.getCurrentPosition() <= (int) pos - 4 && l2Motor.getCurrentPosition() <= (int) pos - 4 && lMotor.getCurrentPosition() <= (int) pos - 4)) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;
            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            lMotor.setPower(leftSpeed);
            l2Motor.setPower(-leftSpeed);

            rMotor.setPower(rightSpeed);
            r2Motor.setPower(-rightSpeed);

            while (r2Motor.getCurrentPosition() <= (int) pos - 4 && rMotor.getCurrentPosition() <= (int) pos - 4 && l2Motor.getCurrentPosition() <= (int) pos - 4 && lMotor.getCurrentPosition() <= (int) pos - 4) {
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", pos, pos);
                telemetry.addData("ActualFront", "%7d:%7d", lMotor.getCurrentPosition(),
                        rMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }
        }
        // Stop all motion;
        power(0);
    }


    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        power(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            power(0);
            onTarget = true;
            return onTarget;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        lMotor.setPower(leftSpeed);
        l2Motor.setPower(leftSpeed);
        rMotor.setPower(rightSpeed);
        r2Motor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void rotateSimple(double power) {
        double leftPower = power;
        double rightPower = power;

        lMotor.setPower(leftPower);
        l2Motor.setPower(leftPower);
        rMotor.setPower(rightPower);
        r2Motor.setPower(rightPower);
        sleep(1000);
        power(0);
    }
    private double checkAngle()
    {
        double anglecorrection, gain = 0.05;
        double angle=getAngle();
        anglecorrection = -angle;
        anglecorrection *= gain;
        return anglecorrection;
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);

        globalAngle = 0;
    }
    private double exponentialTurnPower(double range)
    {
        double exppower = 0.7;
        if(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > Math.abs(range) - 35){
            exppower = 0.25;
        }


        return exppower;
    }
    private void extend(int power)
    {
        if(power < 0)
        {
            while(extender.getCurrentPosition()<0)
            {
                extender.setPower(-1);
            }
        }
    }
    private void turnTray()
    {
        reset();
        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Angle",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle );
        telemetry.update();
        while((Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)) > 100)
        {
            telemetry.addData("Angle",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle );
            telemetry.update();
            lMotor.setPower(1);
            l2Motor.setPower(1);
            rMotor.setPower(-1);
            r2Motor.setPower(-1);
        }
        resetAngle();
        lMotor.setPower(-1);
        l2Motor.setPower(-1);
        rMotor.setPower(-1);
        r2Motor.setPower(-1);
        sleep(500);
        lMotor.setPower(0);
        l2Motor.setPower(0);
        rMotor.setPower(0);
        r2Motor.setPower(0);
    }

    private void rotate(double degreess, double power) {


        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degreess) > 359) degreess = (int) Math.copySign(359, degreess);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.
        pidRotate.reset();
        pidRotate.setInputRange(0, degreess);
        pidRotate.setOutputRange(0, power);
        pidRotate.setSetpoint(degreess);
        pidRotate.setTolerance(0.01);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        double range = degreess;
        if (degreess < Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                lMotor.setPower(power);
                l2Motor.setPower(power);
                rMotor.setPower(-power);
                r2Motor.setPower(-power);
                sleep(100);
            }

            do {
                telemetry.addData("Angle:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Rotation", rotation);
                telemetry.addData("Power", power);
                telemetry.addData("Setpoint", pidRotate.getSetpoint());
                telemetry.addData("Error", pidRotate.getError());
                telemetry.addData("Tolerance", pidRotate.getM_tolerance());
                telemetry.update();
                // power will be - on right turn.
                power = exponentialTurnPower(range);
                lMotor.setPower(power);
                l2Motor.setPower(power);
                rMotor.setPower(-power);
                r2Motor.setPower(-power);
            } while (opModeIsActive() &&( Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(degreess) - 0.5) && exponentialTurnPower(range) != 75);
        } else    // left turn.
            do {
                telemetry.addData("Angle:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Rotation", rotation);
                telemetry.addData("Power", power);
                telemetry.addData("Setpoint", pidRotate.getSetpoint());
                telemetry.addData("Error", pidRotate.getError());
                telemetry.addData("Tolerance", pidRotate.getM_tolerance());
                telemetry.update();
                power = exponentialTurnPower(range); // power will be + on left turn.
                lMotor.setPower(-power);
                l2Motor.setPower(-power);
                rMotor.setPower(power);
                r2Motor.setPower(power);
            } while (opModeIsActive() && (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(degreess) - 2) && exponentialTurnPower(range) != 75);

        // turn the motors off.
        lMotor.setPower(0);
        l2Motor.setPower(0);
        rMotor.setPower(0);
        r2Motor.setPower(0);

        rotation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
        pidRotate.reset();
    }
//    private void rotate(int degrees, double power) {
//        resetAngle();
//        double leftPower, rightPower;
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        if (degrees < 0) {   // turn right.
//            leftPower = -1;
//            rightPower = 1;
//        } else if (degrees > 0) {   // turn left.
//            leftPower = 1;
//            rightPower = -1;
//        } else return;
//
//        // set power to rotate.
//        lMotor.setPower(leftPower);
//        l2Motor.setPower(leftPower);
//        rMotor.setPower(rightPower);
//        r2Motor.setPower(rightPower);
//
//        // rotate until turn is completed.
//        if (degrees > 0) {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) == 0) {
//                telemetry.addData("Target", degrees);
//                telemetry.addData("Current", getAngle());
//                telemetry.addData("Current Raw", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//                telemetry.update();
//                lMotor.setPower(leftPower);
//                l2Motor.setPower(leftPower);
//                rMotor.setPower(rightPower);
//                r2Motor.setPower(rightPower);
//            }
//
//            while (opModeIsActive() && (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < degrees) {
//                telemetry.addData("Target", degrees);
//                telemetry.addData("Current", getAngle());
//                telemetry.addData("Current Raw", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//                telemetry.update();
//                lMotor.setPower(leftPower);
//                l2Motor.setPower(leftPower);
//                rMotor.setPower(rightPower);
//                r2Motor.setPower(rightPower);
//            }
//        } else    // left turn.
//            while (opModeIsActive() && (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > degrees) {
//                telemetry.addData("Target", degrees);
//                telemetry.addData("Current", getAngle());
//                telemetry.addData("Current Raw", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//                telemetry.update();
//                lMotor.setPower(leftPower);
//                l2Motor.setPower(leftPower);
//                rMotor.setPower(rightPower);
//                r2Motor.setPower(rightPower);
//            }
//
//        // turn the motors off.
//        lMotor.setPower(0);
//        l2Motor.setPower(0);
//        rMotor.setPower(0);
//        r2Motor.setPower(0);
//        // wait for rotation to stop.
//        sleep(100);
//        resetAngle();
//    }

    private double getDegrees() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private void rotate2(int degrees, double power) {
        resetAngle();
        double leftPower, rightPower;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees > 0) {   // turn right.
            leftPower = -1;
            rightPower = 1;
        } else if (degrees < 0) {   // turn left.
            leftPower = 1;
            rightPower = -1;
        } else return;

        // set power to rotate.
        lMotor.setPower(leftPower);
        l2Motor.setPower(leftPower);
        rMotor.setPower(rightPower);
        r2Motor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees > 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) == 0) {
                telemetry.addData("Target", degrees);
                telemetry.addData("Current", getAngle());
                telemetry.addData("Current Raw", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }

            while (opModeIsActive() && (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > degrees) {
                telemetry.addData("Target", degrees);
                telemetry.addData("Current", getAngle());
                telemetry.addData("Current Raw", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }
        } else    // left turn.
            while (opModeIsActive() && (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < degrees) {
                telemetry.addData("Target", degrees);
                telemetry.addData("Current", getAngle());
                telemetry.addData("Current Raw", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }

        // turn the motors off.
        lMotor.setPower(0);
        l2Motor.setPower(0);
        rMotor.setPower(0);
        r2Motor.setPower(0);
        // wait for rotation to stop.
        sleep(100);
        resetAngle();
    }

    public void IMUInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode  = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        //IMU will be attached to I2C port on a Core Device Interface Module
        //Configured to be a sensor of type "AdaFruit IMU"
        //named "imu"
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu cal status", imu.getCalibrationStatus().toString());
        telemetry.update();
        sleep(50);
    }
    public double getExponentialPower(double range, double pos, double power)
    {
        double exppower;
        if(Math.abs(pos) < Math.abs(0.5 * range))
        {
            exppower = Math.exp((Math.abs(pos)/(range/48))) / 10 + 0.2;
            exppower = Math.copySign(exppower,power);
            if(Math.abs(exppower) > Math.abs(power))
            {
                exppower = power;
            }
        }
        else {
            exppower = Math.pow(10,(Math.abs(range) - (Math.abs(pos)))/(36)) / 5;
            exppower = Math.copySign(exppower,power);
            if(Math.abs(exppower) > Math.abs(power))
            {
                exppower = power;
            }
        }
        telemetry.addData("Power", exppower);
        telemetry.addData("Range", range);
        telemetry.addData("Pos", pos);
        telemetry.update();
        return exppower;
    }
    public void goForward(double power, double inches) {
        reset();
        double exppower = 0;
        correction = pidDrive.performPID(getAngle());
        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int rpos = rMotor.getCurrentPosition() + (int) (inches / iptback);
        int lpos = lMotor.getCurrentPosition() + (int) (inches / iptback);
        int l2pos = l2Motor.getCurrentPosition() + (int) (inches / iptback);
        int r2pos = r2Motor.getCurrentPosition() + (int) (inches / iptback);
        double Heading = getAngle();
        double range = rpos - rMotor.getCurrentPosition();

        r2Motor.setTargetPosition(r2pos);
        l2Motor.setTargetPosition(l2pos);
        rMotor.setTargetPosition(rpos);
        lMotor.setTargetPosition(lpos);

        while (opModeIsActive() && (r2Motor.getCurrentPosition() <= (int) r2pos - 4 && rMotor.getCurrentPosition() <= (int) rpos - 4 && l2Motor.getCurrentPosition() <= (int) l2pos - 4 && lMotor.getCurrentPosition() <= (int) lpos - 4)) {
            exppower = getExponentialPower(range,r2Motor.getCurrentPosition(),power);
            r2Motor.setPower(exppower + checkAngle());
            l2Motor.setPower(exppower - checkAngle());
            rMotor.setPower(exppower + checkAngle());
            lMotor.setPower(exppower - checkAngle());
        }

        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while(getError(heading) > 4 || getError(heading) < -4)
//        {
//            if (getError(heading) > 4)
//            {
//                lMotor.setPower(0.5);
//                l2Motor.setPower(0.5);
//                rMotor.setPower(0.5);
//                rMotor.setPower(0.5);
//            }
//            if(getError(heading) < -4)
//            {
//                lMotor.setPower(-0.5);
//                l2Motor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//            }
//        }
        l2Motor.setPower(0);
        r2Motor.setPower(0);
        lMotor.setPower(0);
        rMotor.setPower(0);
    }
    public void goBackwards(double power, double inches) {

        reset();
        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int rpos = rMotor.getCurrentPosition() + (int) (inches / iptback);
        int lpos = lMotor.getCurrentPosition() + (int) (inches / iptback);
        int l2pos = l2Motor.getCurrentPosition() + (int) (inches / iptback);
        int r2pos = r2Motor.getCurrentPosition() + (int) (inches / iptback);
        double Heading = getAngle();
        double range = rpos - rMotor.getCurrentPosition();
        double exppower = 0;
        r2Motor.setTargetPosition(-r2pos);
        l2Motor.setTargetPosition(-l2pos);
        rMotor.setTargetPosition(-rpos);
        lMotor.setTargetPosition(-lpos);

        while (opModeIsActive() && (Math.abs(r2Motor.getCurrentPosition()) <= Math.abs((int) r2pos - 4) && Math.abs(rMotor.getCurrentPosition()) <= Math.abs((int) rpos - 4) && Math.abs(l2Motor.getCurrentPosition()) <= Math.abs((int) l2pos - 4) && Math.abs(lMotor.getCurrentPosition()) <= Math.abs((int) lpos - 4))) {
            exppower = getExponentialPower(range,r2Motor.getCurrentPosition(),-power);
            r2Motor.setPower(exppower + checkAngle());
            l2Motor.setPower(exppower - checkAngle());
            rMotor.setPower(exppower + checkAngle());
            lMotor.setPower(exppower - checkAngle());
        }
        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while(getError(heading) > 4 || getError(heading) < -4)
//        {
//            if (getError(heading) > 4)
//            {
//                lMotor.setPower(0.5);
//                l2Motor.setPower(0.5);
//                rMotor.setPower(0.5);
//                rMotor.setPower(0.5);
//            }
//            if(getError(heading) < -4)
//            {
//                lMotor.setPower(-0.5);
//                l2Motor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//            }
//        }
        l2Motor.setPower(0);
        r2Motor.setPower(0);
        lMotor.setPower(0);
        rMotor.setPower(0);
    }
    public void strafeRight(double power, double inches) {
        resetAngle();
        reset();
        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int rpos = rMotor.getCurrentPosition() + (int) (inches / iptback);
        int lpos = lMotor.getCurrentPosition() + (int) (inches / iptback);
        int l2pos = l2Motor.getCurrentPosition() + (int) (inches / iptback);
        int r2pos = r2Motor.getCurrentPosition() + (int) (inches / iptback);
        double Heading = getAngle();


        r2Motor.setTargetPosition(r2pos);
        l2Motor.setTargetPosition(-l2pos);
        rMotor.setTargetPosition(-rpos);
        lMotor.setTargetPosition(lpos);

        while (opModeIsActive() && (r2Motor.getCurrentPosition() <= (int) r2pos - 4 && rMotor.getCurrentPosition() <= (int) rpos - 4 && l2Motor.getCurrentPosition() <= (int) l2pos - 4 && lMotor.getCurrentPosition() <= (int) lpos - 4)) {
            r2Motor.setPower(power + checkAngle());
            l2Motor.setPower(-power - checkAngle());
            rMotor.setPower(-power + checkAngle());
            lMotor.setPower(power - checkAngle());
        }
//        while(getError(heading) > 4 || getError(heading) < -4)
//        {
//            if (getError(heading) > 4)
//            {
//                lMotor.setPower(0.5);
//                l2Motor.setPower(0.5);
//                rMotor.setPower(0.5);
//                rMotor.setPower(0.5);
//            }
//            if(getError(heading) < -4)
//            {
//                lMotor.setPower(-0.5);
//                l2Motor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//            }
//        }
        l2Motor.setPower(0);
        r2Motor.setPower(0);
        lMotor.setPower(0);
        rMotor.setPower(0);
    }
    public void lift(double power, double inches)
    {
        reset();
        liftrMotor.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        liftlMotor.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        double pos = inches/iptback;
        while(opModeIsActive() && Math.abs(liftlMotor.getCurrentPosition()) < Math.abs(pos) && Math.abs(liftrMotor.getCurrentPosition()) < Math.abs(pos))
        {
            liftlMotor.setPower(power);
            liftrMotor.setPower(-power);
        }
        liftrMotor.setPower(0);
        liftlMotor.setPower(0);
    }
    public void strafeLeft(double power, double inches) {
        resetAngle();
        reset();
        double originalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int rpos = rMotor.getCurrentPosition() + (int) (inches / iptback);
        int lpos = lMotor.getCurrentPosition() + (int) (inches / iptback);
        int l2pos = l2Motor.getCurrentPosition() + (int) (inches / iptback);
        int r2pos = r2Motor.getCurrentPosition() + (int) (inches / iptback);
        double Heading = getAngle();


        r2Motor.setTargetPosition(-r2pos);
        l2Motor.setTargetPosition(l2pos);
        rMotor.setTargetPosition(rpos);
        lMotor.setTargetPosition(-lpos);

        while (opModeIsActive() && (r2Motor.getCurrentPosition() <= (int) r2pos - 4 && rMotor.getCurrentPosition() <= (int) rpos - 4 && l2Motor.getCurrentPosition() <= (int) l2pos - 4 && lMotor.getCurrentPosition() <= (int) lpos - 4)) {
            r2Motor.setPower(-power + checkAngle());
            l2Motor.setPower(power - checkAngle());
            rMotor.setPower(power + checkAngle());
            lMotor.setPower(-power - checkAngle());
        }
//        while(getError(heading) > 4 || getError(heading) < -4)
//        {
//            if (getError(heading) > 4)
//            {
//                lMotor.setPower(0.5);
//                l2Motor.setPower(0.5);
//                rMotor.setPower(0.5);
//                rMotor.setPower(0.5);
//            }
//            if(getError(heading) < -4)
//            {
//                lMotor.setPower(-0.5);
//                l2Motor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//                rMotor.setPower(-0.5);
//            }
//        }
        globalAngle = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) - originalAngle;
        l2Motor.setPower(0);
        r2Motor.setPower(0);
        lMotor.setPower(0);
        rMotor.setPower(0);
    }

    public void safeDrive(double pfl, double pbl, double pfr, double pbr) { // Stagger the order to reduce left/right front/back bias at start
        lMotor.setPower(safePower(pfl));
        r2Motor.setPower(safePower(pbr));
        l2Motor.setPower(safePower(pbl));
        rMotor.setPower(safePower(pfr));
    }

    public double safePower(double power) {
        if (power < -1.0) {
            power = -1.0;
        }
        if (power > 1.0) {
            power = 1.0;
        }
        return power;
    }

    public double scaleDiffPower(double power, double diff, double sign) {
        double p = 0.0;
        double p2 = power + (sign * diff);
        p = p2;
        return safePower(p);
    }

    public void angularDrive(double power, double distance, double angle) {
        resetAngle();
        double heading = getDegrees();
        reset();
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double r = power;
        double robotAngle = (angle * Math.PI)/180 + Math.PI/4;
        final double pos1 = (distance/ iptback) * Math.cos(robotAngle);
        final double pos2 = (distance/ iptback) * Math.sin(robotAngle);
        final double pos3 = (distance/ iptback) * Math.sin(robotAngle);
        final double pos4 = (distance/ iptback) * Math.cos(robotAngle);
        double v1;
        double v2;
        double v3;
        double v4;

        while (opModeIsActive() && (Math.abs(r2Motor.getCurrentPosition()) <= Math.abs((int) pos4 - 4) || Math.abs(pos4) <= 0.1) && (Math.abs(rMotor.getCurrentPosition()) <= Math.abs((int) pos2 - 4) || Math.abs(pos2) <= 0.1) && (Math.abs(l2Motor.getCurrentPosition()) <= Math.abs((int) pos3 - 4) || Math.abs(pos3) <= 0.1) && (Math.abs(lMotor.getCurrentPosition()) <= Math.abs((int) pos1 - 4) || Math.abs(pos1) <= 0.1))
        {
            v1 = r * Math.cos(robotAngle);
            v2 = r * Math.sin(robotAngle);
            v3 = r * Math.sin(robotAngle);
            v4 = r * Math.cos(robotAngle);
            r2Motor.setPower(v4 +checkAngle());
            l2Motor.setPower(v3 - checkAngle());
            rMotor.setPower(v2 + checkAngle());
            lMotor.setPower(v1 - checkAngle());
        }
        power(0);
    }


}
