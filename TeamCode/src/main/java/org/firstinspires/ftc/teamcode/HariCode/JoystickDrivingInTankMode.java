// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended.

package org.firstinspires.ftc.teamcode.HariCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive Tank", group="Exercises")
//@Disabled
public class JoystickDrivingInTankMode extends LinearOpMode {
    DcMotor leftDrive, rightDrive;
    float   leftY, rightY;

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
            leftY = gamepad1.left_stick_y * -1;
            rightY = gamepad1.right_stick_y * -1;

            leftDrive.setPower(Range.clip(leftY, -1.0, 1.0));
            rightDrive.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);
            telemetry.update();

            idle();
        }
    }
}