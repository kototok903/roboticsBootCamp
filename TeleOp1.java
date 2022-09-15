package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Remember to comment everything

@TeleOp(name="TeleOp1", group = "TeleOp")
public class TeleOp1 extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    double speed;
    final double MAX_STICK_COORD = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize everything
        initialize();

        // wait for the start button
        waitForStart();

        // main loop
        while (opModeIsActive()) {
            driveset();
            //grabset();
            //armcontroll();
            //shootset();
            //setintake();
            //stepset();
        }
    }


    // Driver with gamepad 1 drives the robot
    // Robot can go in all four directions with the left stick and turn with the right stick
    private void driveset() {
        if (gamepad1.right_bumper) {
            speed = 2.0;
        }
        if (gamepad1.left_bumper) {
            speed = 0.7;
        }
        
        double ls_y = gamepad1.left_stick_y, ls_x = gamepad1.left_stick_x; // -1 - 1
        double rs_x = -gamepad1.right_stick_x; // -1 - 1

        //                   move  strafe    turn
        double frontLeftV  = ls_x - ls_y + 0.8*rs_x;
        double frontRightV = ls_x + ls_y - 0.8*rs_x;
        double backLeftV   = ls_x + ls_y + 0.8*rs_x;
        double backRightV  = ls_x - ls_y - 0.8*rs_x;

        frontLeftV  = frontLeftV  / (2.8 * MAX_STICK_COORD) * speed; // 2.8 = ls_y (1) + ls_x (1) + 0.8rs_x (0.8)
        frontRightV = frontRightV / (2.8 * MAX_STICK_COORD) * speed;
        backLeftV   = backLeftV   / (2.8 * MAX_STICK_COORD) * speed;
        backRightV  = backRightV  / (2.8 * MAX_STICK_COORD) * speed;

        frontLeft.setPower(frontLeftV);
        frontRight.setPower(frontRightV);
        backLeft.setPower(backLeftV);
        backRight.setPower(backRightV);
        
        /*
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
        */
    }


    // Initialization code
    // Robot should not yet move on intitalization
    public void initialize() {
        speed = 0.7;
        
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
