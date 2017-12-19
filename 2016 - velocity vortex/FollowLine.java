package org.firstinspires.ftc.teamcode;

/**
 * Created by JAEGER on 11/25/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Compbot: Follow Line", group="BLUE")
@Disabled
public class FollowLine extends LinearOpMode {

    /* Declare OpMode members. */
    Comp8995 robot   = new Comp8995();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    //LightSensor             lightSensor;      // Primary LEGO Light sensor,
    OpticalDistanceSensor lightSensor;   // Alternative MR ODS sensor

    static final double     WHITE_THRESHOLD = 0.4;  // spans between 0.1 - 0.5 from dark to light
    //static final double     DRIVE_SPEED  = 0.;
    static final double     holdTime = 6.0;

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");                // Primary LEGO Light Sensor
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        //lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getRawLightDetected());
            telemetry.update();
            idle();
        }

        // Start the robot moving forward, and then begin looking for a white line.

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getRawLightDetected());
            telemetry.update();


            //correction = (WHITE_THRESHOLD - lightSensor.getLightDetected());
            if (WHITE_THRESHOLD - lightSensor.getRawLightDetected() <= 0) {
                robot.leftMotor.setPower((.075d) - (WHITE_THRESHOLD - lightSensor.getRawLightDetected()));
                robot.rightMotor.setPower(.075d);

            } else {
                robot.leftMotor.setPower(.075d);
                robot.rightMotor.setPower((.075d) + (WHITE_THRESHOLD - lightSensor.getRawLightDetected()));
            }

        }

        // Stop all motors
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
}
