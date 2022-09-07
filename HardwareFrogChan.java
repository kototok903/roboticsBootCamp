package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
the peropouse of this class is mostly to make our robot more compatable with the examples,
but It odes seem like a usefull feature and maybe a not so usefull one in some cases,
for now Im keeping the hardware map/init stufff in the auto code and TeleOp but that might change.
 */

public class HardwareFrogChan {
    public DcMotor frontRight  = null;
    public DcMotor frontLeft   = null;
    public DcMotor backRight   = null;
    public DcMotor backLeft    = null;
    public DcMotor armmove     = null;
    public DcMotor intake      = null;
    public DcMotor flywheel    = null;
    public Servo   stepper     = null;
    public Servo   woblearm    = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    /**  this is from the pushbot, not fully understood.  */


    /* Constructor */
   public HardwareFrogChan(){


   }
   /**  i don't see why you need an empty constructer but oh well*/

   public void init(HardwareMap ahwMap) {
       // Save reference to Hardware map
       hwMap = ahwMap;



       frontRight = hwMap.get(DcMotor.class, "fr");
       frontLeft = hwMap.get(DcMotor.class, "fl");
       backRight = hwMap.get(DcMotor.class, "br");
       backLeft = hwMap.get(DcMotor.class, "bl");
       armmove = hwMap.get(DcMotor.class, "wm");
       intake = hwMap.get(DcMotor.class, "in");
       flywheel = hwMap.get(DcMotor.class, "fw");
       /**REMEMBER TO SET ENCODER DIRECTON*/

       backRight.setPower(0);
       backLeft.setPower(0);
       frontLeft.setPower(0);
       frontRight.setPower(0);
       armmove.setPower(0);
       intake.setPower(0);
       flywheel.setPower(0);


       flywheel.   setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       backRight.  setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       backLeft.   setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       frontLeft.  setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       frontRight. setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       armmove.    setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       intake.     setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       armmove.    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       //initialize servos
       woblearm = hwMap.get(Servo.class, "wa");
       stepper = hwMap.get(Servo.class, "stp");
       woblearm.setPosition(0.3);//all we need to ever use.
       stepper.setPosition(0.45);












   }








}
