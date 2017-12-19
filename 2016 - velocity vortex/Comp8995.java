package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Comp8995
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  liftMotor   = null;
    public DcMotor  shooterMotor = null;


    public Servo    leftPush    = null;
    public Servo    rightPush   = null;
    public Servo Brush1      = null;
    public Servo Brush2      = null;
    public Servo Conveyor1   = null;
    public Servo Conveyor2   = null;
    public Servo ArmL = null;
    public Servo ArmR = null;

    //public Servo    forkDown    = null;

    /* 2015 Season
    public DcMotor armMotor   = null;
    public DcMotor turboMotor   = null;
    public DcMotor scoopMotor   = null;
    public Servo Right   = null;
    public Servo Left   = null;
    public Servo brush1   = null;
    public Servo brush2   = null;
    public Servo allclearRed   = null;
    public Servo allclearBlue   = null;
    public Servo climber   = null;
    public Servo hang   = null;
    public GyroSensor Gyro   = null;
    public ElapsedTime time   = null; */

    public static final double LEFT_IN_SERVO  = 0.99 ;
    public static final double LEFT_OUT_SERVO = 0.0 ;
    public static final double RIGHT_IN_SERVO  = 0.99 ;
    public static final double RIGHT_OUT_SERVO = 0.0 ;
    public static final double BRUSH1 = 0.5;
    public static final double BRUSH2 = 0.5;
    public static final double CONVEYOR1 = 0.5;
    public static final double CONVEYOR2 = 0.5;
    public static final double LEFTARMOPEN = 0.9;
    public static final double RIGHTARMOPEN = 0.0;
    public static final double LEFTARMCLOSED = 0.0;
    public static final double RIGHTARMCLOSED = 0.9;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Comp8995(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor       = hwMap.dcMotor.get("m1");
        rightMotor      = hwMap.dcMotor.get("m2");
        liftMotor       = hwMap.dcMotor.get("m3");
        shooterMotor    = hwMap.dcMotor.get("m4");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        liftMotor.setPower(0);
        shooterMotor.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftPush = hwMap.servo.get("las1"); // SC1-P1
        leftPush.setPosition(LEFT_IN_SERVO);
        rightPush = hwMap.servo.get("ras2"); // SC2-P1
        rightPush.setPosition(RIGHT_IN_SERVO);
        Brush1 = hwMap.servo.get("lbs3"); // SC1-P2
        Brush1.setPosition(0.5);
        Brush2 = hwMap.servo.get("rbs4"); // SC2-P2
        Brush2.setPosition(0.5);
        Conveyor1 = hwMap.servo.get("lcs5"); // SC1-P3
        Conveyor1.setPosition(0.5);
        Conveyor2 = hwMap.servo.get("rcs6");
        Conveyor2.setPosition(0.5); // SC2-P3
        ArmL = hwMap.servo.get("arml"); //Cap Ball Arm Left
        ArmL.setPosition(LEFTARMOPEN);
        ArmR = hwMap.servo.get("armr"); //Cap Ball Arm Right
        ArmR.setPosition(RIGHTARMOPEN);
    }







    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

