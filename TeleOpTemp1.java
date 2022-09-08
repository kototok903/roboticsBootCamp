package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Remember to comment everything

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize everything
        initialize();

        // wait for the start button
        waitForStart();

        // main loop
        while (opModeIsActive()) {
            driveset();
            grabset();
            armcontroll();
            shootset();
            setintake();
            stepset();
        }
    }


    // Driver with gamepad 1 drives the robot
    // Robot can go in all four directions with the left stick and turn with the right stick
    private void driveset() {
        double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x); // we might have to take out the negatives
        double robotAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;

        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeft.setPower(v1*1.41); // multiply them all by 1.414
        frontRight.setPower(v2*1.41); // why?
        backLeft.setPower(v3*1.41);
        backRight.setPower(v4*1.41);
    }


    // Initialization code
    // Robot should not yet move on intitalization
    public void initialize() {
        // initialize drive train motors
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}