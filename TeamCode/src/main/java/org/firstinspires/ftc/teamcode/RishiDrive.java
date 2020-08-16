package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name="RishiDrive", group="Square")
//@Disabled
public class RishiDrive extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        for (int i=0; i<4; i++)
        {
            leftMotor.setPower(0.75);
            rightMotor.setPower(0.75);

            sleep(2000);

            leftMotor.setPower(-0.75);
            rightMotor.setPower(0.75);

            sleep(450);
        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
