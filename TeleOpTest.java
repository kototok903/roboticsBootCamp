package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpTest extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor armmove;
    DcMotor intake;
    DcMotor stepper;
    DcMotor flywheel;
    Servo woblearm;
    boolean previousRb = false;
    boolean previousLb = false;
    boolean previousA = false;
    boolean previousB = false;
    boolean previousCross = false;
    boolean armManual = true;
    @Override
    //remember to comment you slop.
    public void runOpMode() throws InterruptedException {

        initialize();
        //initialize evrything,

        waitForStart();
        //wait for the start button.

        //do what we want to do(where stuff goes wrong)
        while (opModeIsActive()) {
            driveset();
            grabset();
            armcontroll();
            shootset();
            setintake();
            stepset();
            /*setmode();
            //this is mostly so we have a working prototype which lets us use the
            if (armManual == true) {
            armcontroll();
            }
            else {
            testarmadd();
            testarmsub();
            grabmove();
            }
            //armmove.setTargetPosition(1);
            //grabmove();
            */
        }


    }
    // the driver with gamepad 1 drives the robot,
//the robot can go in all four directions with the left stick and turn with the right stick.
    private void driveset() {
        double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x); //we might have to take out the negatives.
        double robotAngle = Math.atan2( gamepad1.left_stick_x,gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;//we may not need the final.
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeft.setPower(v1*1.41);//multiply them all by 1.414
        frontRight.setPower(v2*1.41);
        backLeft.setPower(v3*1.41);
        backRight.setPower(v4*1.41);
    }

    //the driver with gamepad 2 controlls the servo with right bumper to switch between open and closed..
    private void grabset(){
        boolean rb = gamepad2.right_bumper;


        if(rb && !previousRb){
            if(woblearm.getPosition() == 1.0){
                woblearm.setPosition(0.0);
            }
            else{
                woblearm.setPosition(1.0);
            }
        }
        previousRb = rb;
    }
    // the driver with gampad 2 uses the y and b buttons to move the arm up and down.
    private void armcontroll(){
        if(gamepad2.y) {
            armmove.setPower(0.6);
        }
        else if(gamepad2.b){
            armmove.setPower(-0.6);
        }
        else {
            armmove.setPower(0.0);

        }
    }
    // the driver with gamepad 2 uses the right trigerr to activate the motor and shoot disks.
    private void shootset(){
        if(gamepad2.right_trigger > 0.25){
            flywheel.setPower(1.0);
        }else {
            flywheel.setPower(0.0);
        }
    }
    //the
    private void setintake(){
        if(gamepad1.left_trigger > 0.25){
            intake.setPower(1.0);
        }else {
            intake.setPower(0.0);
        }
    }//drive train does this.
    //the secondary step motor in th eintake is controlled by gampad 2.
    private void stepset(){
        if(gamepad2.x){
            stepper.setPower(0.7);
        }
        else {
            stepper.setPower(0.0);
        }
    }
    //initialization code. robot should not yet move on intitalization.
    public void initialize() {

        //initialize servos
        woblearm = hardwareMap.get(Servo.class, "wa");
        //woblearm.setPosition(1);//all we need to ever use.

        //initialize the wobble arm motor.
        armmove = hardwareMap.get(DcMotor.class, "wm");
        armmove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //we set it so that the position the motor is in does not go any farther back when we reset it, and so that the motor hopefully does less work holding up the goal.
        armmove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize the intake and output motors
        intake = hardwareMap.get(DcMotor.class, "in");
        stepper = hardwareMap.get(DcMotor.class,"stp");
        flywheel = hardwareMap.get(DcMotor.class, "fw");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stepper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stepper.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        //initialize drive train motors,
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
    /*
    private void  grabmove(){
        boolean lb = gamepad1.left_bumper;

        if(lb && !previousLb){
            if(armmove.getTargetPosition() == 90){//90 is probably not the bast value.
                armmove.setTargetPosition(0);//0 is probably not the right value, in order to use 0 we would have to hold it in the same position we init at.
            }
            else{
                armmove.setTargetPosition(90);//90 is probably not the best value.
            }
        }
        previousLb = lb;
    }
    //swich between manual forward and back and less well tested systems using set target position.
    private void setmode (){
        if(gamepad1.cross && !previousCross){
            armManual=!armManual;
            if(armManual){
                armmove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else {
                armmove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
    //manual contoll of the arm.


    private void moveForeward() {
        //find the value the gamepad joystick reads.
        double power = gamepad1.right_stick_y;
        //power 1 to -1
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    //we dont need this anymore

    //a manual test program that lets you manualy use set trget mode. split into a forward and backwards based on two buttons.
    private void testarmadd(){
        boolean Apress = gamepad1.a;
        if(Apress && ! previousA){
            int next = armmove.getTargetPosition() + 10;
            armmove.setTargetPosition(next);
        }
        previousA = Apress;
    }
    private void testarmsub(){
        boolean Bpress = gamepad1.b;
        if(Bpress && ! previousB){
            int next = armmove.getTargetPosition() + 10;
            armmove.setTargetPosition(next);
        }
        previousB = Bpress;
    }
    */
}