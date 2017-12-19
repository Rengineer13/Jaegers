/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueRight", group="Blue")
//@Disabled
public class BlueRight extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum8995         robot   = new Mecanum8995();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    ColorSensor colorSensor;

    public static final String TAG ="Vuforia VuMark Sample";

    RelicRecoveryVuMark KEY;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    @Override public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AReFsyn/////AAAAGW2uexIkF0dEv1zItq11aHs7YS7+LHppuvZwcNi95vyt+zvMLlAlj8z1zu3p0JZsz8BwPpbQrvbJei3a6v5+ryun6brZvK/YUVW6RcVxA6eaVsUGsgSMCOrzSYzAAPSc/eidRsMs1EMcbk4LkfPp8qWOaTTvUKpyPAZoF5ayTCLp0lnjt3Zx3siNvxg7Xvevtxt7TMw7XYNQVgEC66k4pF4Hgw0b7nssDH2CmPJu0fjurZ9JxPWcK0PcMLACXtP4/bIO5067uvkcxa/C5F9SEKFebD1/d5gQcm8xtD1us7w8L+rw4bzdm4s6C6s+5TAWHZJD9HiSiqQyvNHMaU3xXrPVuLq3zSrkb99YKGoVgJmj";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        ElapsedTime     runtime = new ElapsedTime();
        relicTrackables.activate();
        while (opModeIsActive() && runtime.seconds() < 1) {
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                KEY = vuMark;



                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                //telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                /*if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }*/
            }
            else {
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
            }
        }

        robot.squeezeServo1.setPosition(Mecanum8995.CLOSE_1);
        robot.squeezeServo2.setPosition(Mecanum8995.CLOSE_2);
        robot.squeezeServo3.setPosition(Mecanum8995.CLOSE_2);
        robot.squeezeServo4.setPosition(Mecanum8995.CLOSE_1);
        liftArm(0.5, -6.0, 2.0);
        armDown(1.0);
        readColor(0.25);
        if (KEY == RelicRecoveryVuMark.LEFT) {
            gyroDrive(0.5, 25.0, 0);
            gyroTurn(0.5, -90.0);
            gyroHold(0.5, -90.0, 0.1);
            gyroDrive(0.5, -10.0, -90.0);
            armOpen(0.5);
        }
        else {
        }
        if (KEY == RelicRecoveryVuMark.RIGHT) {
            gyroDrive(0.5, 39.0, 0);
            gyroTurn(0.5, -90.0);
            gyroHold(0.5, -90.0, 0.1);
            gyroDrive(0.5, -10.0, -90.0);
            armOpen(0.5);

        }
        else {
        }
        if (KEY == RelicRecoveryVuMark.CENTER) {
            gyroDrive(0.5, 32.0, 0);
            gyroTurn(0.5, -90.0);
            gyroHold(0.5, -90.0, 0.1);
            gyroDrive(0.5, -10.0, -90.0);
            armOpen(0.5);
        }
        else{
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /*
       @param holdTime   Length of time (in seconds) to hold the specified heading.
               */
    /**

     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void readColor( double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            if (colorSensor.red() > 4){
                gyroDrive(0.5, -2.0, 0.0);
                robot.armservo.setPosition(Mecanum8995.ARM_UP);
                gyroDrive(0.5, 4.0, 0.0);
            }
            else {
                gyroDrive(0.5, 2.0, 0.0);
                robot.armservo.setPosition(Mecanum8995.ARM_UP);
            }
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.update();
        }

        // Stop all motion;
        robot.FrontLeftMotor.setPower(0);
        robot.FrontRightMotor.setPower(0);
        robot.BackLeftMotor.setPower(0);
        robot.BackRightMotor.setPower(0);
    }

    /*
    @param holdTime   Length of time (in seconds) to hold the specified heading.
            */
    public void armDown(double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)){

            robot.armservo.setPosition(Mecanum8995.ARM_DOWN);

        }

    }

    public void armUp(double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)){

            robot.armservo.setPosition(Mecanum8995.ARM_UP);

        }

    }


    public void liftArm(double speed,
                        double liftInches,
                        double timeoutS) {
        int newLiftTarget;
        ElapsedTime holdTimer = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.LiftMotor.getCurrentPosition() + (int) (liftInches * COUNTS_PER_INCH);

            robot.LiftMotor.setTargetPosition(newLiftTarget);


            // Turn On RUN_TO_POSITION
            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            holdTimer.reset();
            robot.LiftMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (holdTimer.seconds() < timeoutS) &&
                    (robot.LiftMotor.isBusy())) {

                // Display it for the driver.
                robot.LiftMotor.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            robot.LiftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    /**

     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void armOpen(double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {

            robot.squeezeServo1.setPosition(Mecanum8995.OPEN_1);
            robot.squeezeServo2.setPosition(Mecanum8995.OPEN_2);
            robot.squeezeServo3.setPosition(Mecanum8995.OPEN_2);
            robot.squeezeServo4.setPosition(Mecanum8995.OPEN_1);

        }
    }



    public void gyroStrafe (double speed, double distance, double angle) {
        int     newLeftTarget;
        int     newRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newBackLeftTarget = robot.FrontLeftMotor.getCurrentPosition() - moveCounts;
            newLeftTarget = robot.BackLeftMotor.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.FrontRightMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.BackRightMotor.getCurrentPosition() - moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.FrontLeftMotor.setTargetPosition(newLeftTarget);
            robot.BackLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.FrontRightMotor.setTargetPosition(newRightTarget);
            robot.BackRightMotor.setTargetPosition(newBackRightTarget);

            robot.FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FrontLeftMotor.setPower(speed);
            robot.BackLeftMotor.setPower(speed);
            robot.FrontRightMotor.setPower(speed);
            robot.BackRightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.FrontLeftMotor.isBusy() && robot.FrontRightMotor.isBusy() && robot.BackLeftMotor.isBusy() && robot.BackRightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.FrontLeftMotor.setPower(leftSpeed);
                robot.BackLeftMotor.setPower(leftSpeed);
                robot.FrontRightMotor.setPower(rightSpeed);
                robot.BackRightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.FrontLeftMotor.getCurrentPosition(),
                        robot.FrontRightMotor.getCurrentPosition(), robot.BackLeftMotor.getCurrentPosition(), robot.BackRightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.FrontLeftMotor.setPower(0);
            robot.FrontRightMotor.setPower(0);
            robot.BackLeftMotor.setPower(0);
            robot.BackRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.FrontLeftMotor.getCurrentPosition() + moveCounts;
            newLeftTarget = robot.BackLeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.FrontRightMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.BackRightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.FrontLeftMotor.setTargetPosition(newLeftTarget);
            robot.BackLeftMotor.setTargetPosition(newLeftTarget);
            robot.FrontRightMotor.setTargetPosition(newRightTarget);
            robot.BackRightMotor.setTargetPosition(newRightTarget);

            robot.FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FrontLeftMotor.setPower(speed);
            robot.BackLeftMotor.setPower(speed);
            robot.FrontRightMotor.setPower(speed);
            robot.BackRightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.FrontLeftMotor.isBusy() && robot.FrontRightMotor.isBusy() && robot.BackLeftMotor.isBusy() && robot.BackRightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.FrontLeftMotor.setPower(leftSpeed);
                robot.BackLeftMotor.setPower(leftSpeed);
                robot.FrontRightMotor.setPower(rightSpeed);
                robot.BackRightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.FrontLeftMotor.getCurrentPosition(),
                        robot.FrontRightMotor.getCurrentPosition(), robot.BackLeftMotor.getCurrentPosition(), robot.BackRightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.FrontLeftMotor.setPower(0);
            robot.FrontRightMotor.setPower(0);
            robot.BackLeftMotor.setPower(0);
            robot.BackRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.FrontRightMotor.setPower(0);
        robot.FrontLeftMotor.setPower(0);
        robot.BackLeftMotor.setPower(0);
        robot.BackRightMotor.setPower(0);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.FrontLeftMotor.setPower(leftSpeed);
        robot.FrontRightMotor.setPower(rightSpeed);
        robot.BackLeftMotor.setPower(leftSpeed);
        robot.BackRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
