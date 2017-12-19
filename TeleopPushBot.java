package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Thread.sleep;

/**
 * Created by JAEGER on 2/9/2017.
 */

@TeleOp(name="TeleopPushBot", group="Teleop")
@Disabled
public class TeleopPushBot extends OpMode {

    /* Declare OpMode members. */
    Pushbot8995 robot       = new Pushbot8995(); // use the class created to define a Pushbot's hardware


    double      brushPower      =1.0;
    boolean     inputState;
    boolean     outputState;
    double      conveyorPower   =1.0;
    boolean     Reverse         = false;
    double brush1Delta = 0.5;
    double brush1Position;
    double brush2Delta = 0.5;
    double brush2Position;
    double conveyor1Delta = 0.5;
    double conveyor1Position;
    double conveyor2Delta = 0.5;
    double conveyor2Position;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.leftPush.setPosition(Comp8995.LEFT_IN_SERVO);
        //robot.rightPush.setPosition(Comp8995.RIGHT_IN_SERVO);
        //robot.Brush1.setPosition(0.5);
       // brush1Position = 0.5;
       // robot.Brush2.setPosition(0.5);
        //brush2Position = 0.5;

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Say", "Hello Driver");    //
        //updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        outputState = gamepad1.a;
        //double liftUp;
        //double liftDown;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        /*left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);*/

        /*float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        right = (float)scaleInput(right);
        left = (float)scaleInput(left);*/

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);


        //Use the Right trigger to raise the lift and the left trigger to lower the lift
        //liftUp = -gamepad1.right_trigger;
        //liftDown = -gamepad1.left_trigger;
        //robot.liftMotor.setPower(liftUp);
        //robot.leftMotor.setPower(-liftDown);*/

        /*if(gamepad1.x) {

            if (Reverse == true) {

                Reverse = false;
            } else if (Reverse == false) {

                Reverse = true;

            }
            try {
                sleep(250);
            } catch (InterruptedException e) {

            }
        }

        if (Reverse == true) {

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

        }
        else if (Reverse == false){

            robot.leftMotor.setPower(-right);
            robot.rightMotor.setPower(-left);

        }*/

        /*if(!(gamepad1.right_trigger == 0)){

            robot.liftMotor.setPower(gamepad1.right_trigger);

        }
        else if(!(gamepad1.left_trigger == 0)){

            robot.liftMotor.setPower(-gamepad1.left_trigger);

        }
        else {
            robot.liftMotor.setPower(0);
        }



        if (gamepad1.right_bumper)
            robot.rightPush.setPosition(Comp8995.RIGHT_OUT_SERVO);
        else
            robot.rightPush.setPosition(Comp8995.RIGHT_IN_SERVO);
        //Use the Left Gamepad 1 bumber to put out the Left servo while pressed and retract when release

        if (gamepad1.left_bumper)
            robot.leftPush.setPosition(Comp8995.LEFT_OUT_SERVO);
        else
            robot.leftPush.setPosition(Comp8995.LEFT_IN_SERVO);

        //Use Gamepad 1 when A is pressed to turn on the shooter motor to shoot debris on/off
        if (gamepad2.y) {              // Full Power Shooter Motors
            robot.shooterMotor.setPower(0.60);
        }

        if (gamepad2.x) {              // Partial Power Shooter Motors
            robot.shooterMotor.setPower(0.50);
        }

        if (gamepad2.b){             // Stop Shooter Motors
            robot.shooterMotor.setPower(0.0);
        }

        if (gamepad2.a){        //Reverse Shooter Motors
            robot.shooterMotor.setPower(-0.25);
        }

        if(gamepad2.right_bumper) {
            brush1Position += brush1Delta;
            brush2Position -= brush2Delta;
        }
        else{
            brush1Position = brush1Delta;
            brush2Position = brush2Delta;
        }


        if(gamepad2.left_bumper) {
            brush1Position -= brush1Delta;
            brush2Position += brush2Delta;
        }

        if(gamepad2.dpad_down) {
            conveyor1Position -= conveyor1Delta;
            conveyor2Position += conveyor2Delta;
        }
        else{
            conveyor1Position = conveyor1Delta;
            conveyor2Position = conveyor2Delta;
        }

        if(gamepad2.dpad_up) {
            conveyor1Position += conveyor1Delta;
            conveyor2Position -= conveyor2Delta;
        }

        robot.Brush1.setPosition(brush1Position);
        robot.Brush2.setPosition(brush2Position);
        robot.Conveyor1.setPosition(conveyor1Position);
        robot.Conveyor2.setPosition(conveyor2Position);
*/
        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);

        updateTelemetry(telemetry);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    double scaleInput (double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}

