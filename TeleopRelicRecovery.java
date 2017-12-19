package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by Jaegers8995 on 9/21/2017.
 */


@TeleOp(name="TeleopRelicRecovery", group="Teleop")
//@Disabled
public class TeleopRelicRecovery  extends OpMode {

    Mecanum8995 robot  = new Mecanum8995();

    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        float strafe = -gamepad1.right_stick_x;

        robot.FrontLeftMotor.setPower(left);
        robot.BackLeftMotor.setPower(left);

        robot.FrontRightMotor.setPower(right);
        robot.BackRightMotor.setPower(right);

        robot.FrontLeftMotor.setPower(-strafe);
        robot.BackLeftMotor.setPower(strafe);
        robot.FrontRightMotor.setPower(strafe);
        robot.BackRightMotor.setPower(-strafe);


        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        //telemetry.addData("strafe", "%.2f", strafe);
        updateTelemetry(telemetry);
    }
}
