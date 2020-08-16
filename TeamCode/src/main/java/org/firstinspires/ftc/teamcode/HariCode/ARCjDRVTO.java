package org.firstinspires.ftc.teamcode.HariCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ARCjDRVTO", group="TeleOP")
public class ARCjDRVTO extends LinearOpMode {
    DcMotor leftDrive, rightDrive;
    float   leftPower, rightPower, xValue, yValue;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
            yValue = gamepad1.left_stick_y * -1;
            xValue = gamepad1.left_stick_x * -1;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            leftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
            rightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));

            if(yValue<0.9)
                if(-0.9<yValue){
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                }


            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();

            idle();
        }
    }
}
