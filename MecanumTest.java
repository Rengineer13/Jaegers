package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.qualcomm.robotcore.util.Range.scale;
import static java.lang.Math.abs;

/**
 * Created by Jaegers8995 on 9/21/2017.
 */


@TeleOp(name="Mecanum Test", group="Teleop")
//@Disabled
public class MecanumTest  extends OpMode {

    Mecanum8995 robot = new Mecanum8995();

    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        double Speed = -gamepad1.right_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = gamepad1.right_stick_x;
        double MAX_SPEED = 1.0;
        holonomic(Speed, Turn, Strafe, MAX_SPEED);
    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED) {

//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range


        robot.FrontLeftMotor.setPower(scale((scaleInput(Speed) + scaleInput(Turn) + scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));

        /*if (robot.FrontLeftMotor != null) {
            robot.FrontLeftMotor.setPower(scale(((Speed) + (Turn) + (Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }*/
        if (robot.BackLeftMotor != null) {
            robot.BackLeftMotor.setPower(scale((scaleInput(Speed) + scaleInput(Turn) - scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        robot.FrontRightMotor.setPower(scale((scaleInput(Speed) - scaleInput(Turn) - scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        /*if (robot.FrontRightMotor != null) {
            robot.FrontRightMotor.setPower(scale(((Speed) - (Turn) - (Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }*/
        if (robot.BackRightMotor != null) {
            robot.BackRightMotor.setPower(scale((scaleInput(Speed) - scaleInput(Turn) + scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
    }


    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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
