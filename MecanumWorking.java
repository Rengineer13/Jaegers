package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * Created by Jaegers8995 on 9/22/2017.
 */


@TeleOp(name="Mecanum Working", group = "Teleop")
//@Disabled
public class MecanumWorking extends OpMode {

    Mecanum8995 robot = new Mecanum8995();
    ColorSensor colorSensor;
    boolean     Reverse         = false;

    @Override
    public void init() {

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        robot.init(hardwareMap);
        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        colorSensor.enableLed(false);

    }

    @Override
    public void start() {
        robot.squeezeServo1.setPosition(Mecanum8995.OPEN_1);
        robot.squeezeServo2.setPosition(Mecanum8995.OPEN_1);
        robot.squeezeServo3.setPosition(Mecanum8995.OPEN_1);
        robot.squeezeServo4.setPosition(Mecanum8995.OPEN_1);
    }

    @Override
    public void loop() {

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = (r * Math.cos(robotAngle) + rightX);
        final double v2 = (r * Math.sin(robotAngle) - rightX);
        final double v3 = (r * Math.sin(robotAngle) + rightX);
        final double v4 = (r * Math.cos(robotAngle) - rightX);


        if(gamepad1.left_stick_button) {

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
            robot.FrontLeftMotor.setPower(v1*0.5);
            robot.FrontRightMotor.setPower(v2*0.5);
            robot.BackLeftMotor.setPower(v3*0.5);
            robot.BackRightMotor.setPower(v4*0.5);

        }
        else if (Reverse == false){
            robot.FrontLeftMotor.setPower(v1*1.4);
            robot.FrontRightMotor.setPower(v2*1.4);
            robot.BackLeftMotor.setPower(v3*1.4);
            robot.BackRightMotor.setPower(v4*1.4);

        }


        if (gamepad1.a) {
            robot.squeezeServo1.setPosition(.35);
            robot.squeezeServo2.setPosition(Mecanum8995.CLOSE_2);
            robot.squeezeServo3.setPosition(Mecanum8995.CLOSE_2);
            robot.squeezeServo4.setPosition(Mecanum8995.CLOSE_1);
        }

        if (gamepad1.b){
            robot.squeezeServo1.setPosition(Mecanum8995.OPEN_1);
            robot.squeezeServo2.setPosition(Mecanum8995.OPEN_2);
            robot.squeezeServo3.setPosition(Mecanum8995.OPEN_2);
            robot.squeezeServo4.setPosition(Mecanum8995.OPEN_1);
        }

        if (gamepad1.y){
            robot.squeezeServo1.setPosition(0.2);
            robot.squeezeServo2.setPosition(0.8);
            robot.squeezeServo3.setPosition(0.75);
            robot.squeezeServo4.setPosition(0.25);
        }


        if (gamepad1.right_trigger > 0.2) {
            robot.LiftMotor.setPower(-gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0.2){
            robot.LiftMotor.setPower(gamepad1.left_trigger);
        }
        else{
            robot.LiftMotor.setPower(0.0);
        }

        telemetry.addData("Power Left Front:", robot.FrontLeftMotor.getPower());
        telemetry.addData("Power Right Front:", robot.FrontRightMotor.getPower());
        telemetry.addData("Power Left Back:", robot.BackLeftMotor.getPower());
        telemetry.addData("Power Right Back:", robot.BackRightMotor.getPower());



    }
    @Override
    public void stop() {
    }

}
