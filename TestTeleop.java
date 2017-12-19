package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Jaegers on 8/31/2017.
 */

@TeleOp(name = "TestTeleop", group = "TeleOp")
//@Disabled
public class TestTeleop extends LinearOpMode {

    Pushbot8995 robot = new Pushbot8995();

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;

        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.

            if(gamepad1.a){
                robot.leftMotor.setPower(1.0);
            }
            else {
                robot.leftMotor.setPower(0.0);
            }

            if (gamepad1.b){
                robot.rightMotor.setPower(1.0);
            }
            else{
                robot.rightMotor.setPower(0.0);
            }

            //robot.leftMotor.setPower(left);
            //robot.rightMotor.setPower(right);

            //telemetry.addData("left",  "%.2f", left);
            //telemetry.addData("right", "%.2f", right);
            telemetry.update();

            robot.waitForTick(40);
        }
    }
}
