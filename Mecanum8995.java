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
public class Mecanum8995
{
    /* Public OpMode members. */
    public DcMotor FrontLeftMotor  = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor BackLeftMotor   = null;
    public DcMotor BackRightMotor  = null;
    public DcMotor LiftMotor       = null;
    public Servo   armservo = null;
    public Servo   squeezeServo1 = null;
    public Servo   squeezeServo2 = null;
    public Servo   squeezeServo3 = null;
    public Servo   squeezeServo4 = null;

    public static final double ARM_UP = 1.0;
    public static final double ARM_DOWN = 0.0;
    public static final double OPEN_INIT_1 = 0.65;
    public static final double OPEN_INIT_2 = 0.35;
    public static final double OPEN_1 = 0.5;
    public static final double OPEN_2 = 0.5;
    public static final double CLOSE_1 = 0.28;
    public static final double CLOSE_2 = 0.72;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Mecanum8995(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrontLeftMotor     = hwMap.dcMotor.get("FL");
        FrontRightMotor    = hwMap.dcMotor.get("FR");
        BackLeftMotor      = hwMap.dcMotor.get("BL");
        BackRightMotor     = hwMap.dcMotor.get("BR");
        LiftMotor          = hwMap.dcMotor.get("LM");

        FrontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        FrontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        BackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);
        LiftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armservo =  hwMap.servo.get("s1");
        squeezeServo1 = hwMap.servo.get("s2");
        squeezeServo2  = hwMap.servo.get("s3");
        squeezeServo3  = hwMap.servo.get("s4");
        squeezeServo4  = hwMap.servo.get("s5");
        squeezeServo1.setPosition(OPEN_INIT_1);
        squeezeServo2.setPosition(OPEN_INIT_2);
        squeezeServo3.setPosition(OPEN_INIT_2);
        squeezeServo4.setPosition(OPEN_INIT_1);
        armservo.setPosition(ARM_UP);


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

