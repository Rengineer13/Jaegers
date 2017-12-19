package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a legacy (NXT-compatible) Light Sensor.
 * It assumes that the light sensor is configured with a name of "sensor_light".
 *
 * You can use the X button on gamepad1 to turn Toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: LEGO light", group = "Sensor")
@Disabled
public class SensorLEGOLight extends LinearOpMode {

    Comp8995 robot  = new Comp8995();
    //LightSensor lightSensor;  // Hardware Device Object
    OpticalDistanceSensor lightSensor;

    static final double         WHITE_THRESHOLD = 0.4;

    @Override
    public void runOpMode() {

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");                // Primary LEGO Light Sensor

        // Set the LED state in the beginning.

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // check the status of the x button .
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if ((bCurrState == true) && (bCurrState != bPrevState))  {

                if (WHITE_THRESHOLD - lightSensor.getRawLightDetected() <= 0) {
                    robot.leftMotor.setPower((.075d) - (WHITE_THRESHOLD - lightSensor.getRawLightDetected()));
                    robot.rightMotor.setPower(.075d);

                } else {
                    robot.leftMotor.setPower(.075d);
                    robot.rightMotor.setPower((.075d) + (WHITE_THRESHOLD - lightSensor.getRawLightDetected()));
                }

            }
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Raw", lightSensor.getRawLightDetected());
            telemetry.addData("Normal", lightSensor.getLightDetected());

            telemetry.update();
        }
    }
