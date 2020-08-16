package org.firstinspires.ftc.teamcode.HariCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Drive in a Circle", group="Exercises")
//@Disabled
public class DRVCIRA extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(500);

        leftDrive.setPower(0.75);
        rightDrive.setPower(0.5);

        sleep(16000);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }

}
