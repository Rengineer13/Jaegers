package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

/**
 * Created by Jaegers8995 on 9/29/2017.
 */

@TeleOp(name = "Teleop Gyro", group = "Teleop")
//@Disabled
public class TeleopGyro extends LinearOpMode {

    GyroTest robot = new GyroTest();
    ModernRoboticsI2cGyro gyro = null;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double     Servo_Offset            = 0.01;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        gyro.calibrate();

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && gyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        waitForStart();
        telemetry.log().clear();
        telemetry.log().add("Press Start");

        while (opModeIsActive()) {

            if ((gyro.getHeading() > 0) && (gyro.getHeading() < 65)){
                robot.armservo.setPosition(((0.0048395581)*gyro.getHeading()) + .2459985972);
            }

            if ((gyro.getHeading() > 66) && (gyro.getHeading() < 204)){
                robot.armservo.setPosition(0.5);
            }

            if ((gyro.getHeading() > 205) && (gyro.getHeading() < 275)){
                robot.armservo.setPosition(((.0043688834)*gyro.getHeading()) - 0.4760586096);
            }


            telemetry.addData("Heading Value:", (gyro.getHeading()));
            telemetry.addData("Servo Position:", (robot.armservo.getPosition()));
            telemetry.update();

        }
    }
}
