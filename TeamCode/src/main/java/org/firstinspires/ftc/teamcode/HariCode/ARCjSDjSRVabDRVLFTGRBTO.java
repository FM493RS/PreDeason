package org.firstinspires.ftc.teamcode.HariCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ARCjSDjSRVabDRVLFTGRBTO", group="TeleOP")
public class ARCjSDjSRVabDRVLFTGRBTO extends LinearOpMode {
    DcMotor leftDrive, rightDrive, leftLift, rightLift;
    Servo leftBar, rightBar;
    float   leftPower, rightPower, xValue, yValue, liftY;
    double leftBarPos, rightBarPos;
    double MIN_POSITION = 0, MAX_POSITION = 1;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftLift = hardwareMap.dcMotor.get("left_lift");
        rightLift = hardwareMap.dcMotor.get("right_lift");
        leftBar=hardwareMap.servo.get("left_bar");
        rightBar=hardwareMap.servo.get("right_bar");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        //rightBar.setDirection(Servo.Direction.REVERSE);


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        leftBarPos = 0.5;
        rightBarPos = 0.5;

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
            yValue = gamepad1.left_stick_y * -1;
            xValue = gamepad1.left_stick_x * -1;
            liftY = gamepad1.right_stick_y* -1;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            if (gamepad1.a && leftBarPos > MIN_POSITION) leftBarPos -= .01;
            if (gamepad1.a && rightBarPos < MAX_POSITION) rightBarPos += .01;

            if (gamepad1.b && leftBarPos < MAX_POSITION) leftBarPos += .01;
            if (gamepad1.b && rightBarPos > MIN_POSITION) rightBarPos -= .01;


            leftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
            rightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
            leftLift.setPower(0.25 * Range.clip(liftY, -1.0, 1.0));
            rightLift.setPower(0.25* Range.clip(liftY, -1.0, 1.0));
            leftBar.setPosition(Range.clip(leftBarPos, MIN_POSITION, MAX_POSITION));
            rightBar.setPosition(Range.clip(rightBarPos, MIN_POSITION, MAX_POSITION));

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
