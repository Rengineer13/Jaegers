package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Reed.Hancock on 10/14/2015.
 */
public class LinearTest extends LinearOpMode {



    DcMotor motorRight;
    DcMotor motorLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();


        motorLeft.setPower(1.0);
        motorRight.setPower(1.0);
        sleep(3000);

        motorLeft.setPowerFloat();
        motorRight.setPowerFloat();









    }


}
