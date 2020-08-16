package org.firstinspires.ftc.teamcode.HariCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Drive Encoder", group="Exercises")
//@Disabled
public class DRVENA extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder count kept by left motor.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set right motor to run without regard to an encoder.

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.

        leftDrive.setTargetPosition(5000);
        rightDrive.setTargetPosition(5000);
        // set both motors to 25% power. Movement will start.

        leftDrive.setPower(0.25);
        rightDrive.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && leftDrive.isBusy() && rightDrive.isBusy()) {
            telemetry.addData("encoder-fwd", leftDrive.getCurrentPosition() + "  busy=" + leftDrive.isBusy());
            telemetry.addData("encoder-fwd", rightDrive.getCurrentPosition() + "  busy=" + rightDrive.isBusy());
            telemetry.update();
            idle();
        }


        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        // wait 5 sec so you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5) {
            telemetry.addData("encoder-fwd-end", leftDrive.getCurrentPosition() + "  busy=" + leftDrive.isBusy());
            telemetry.addData("encoder-fwd-end", rightDrive.getCurrentPosition() + "  busy=" + rightDrive.isBusy());
            telemetry.update();
            idle();
        }

        // Now back up to starting point. In this example instead of
        // having the motor monitor the encoder, we will monitor the encoder ourselves.

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setPower(-0.25);
        rightDrive.setPower(-0.25);

        while (opModeIsActive() && leftDrive.getCurrentPosition() > 0 && rightDrive.getCurrentPosition() > 0) {
            telemetry.addData("encoder-back", leftDrive.getCurrentPosition());
            telemetry.addData("encoder-back", rightDrive.getCurrentPosition());

            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        // wait 5 sec so you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5) {
            telemetry.addData("encoder-back-end", leftDrive.getCurrentPosition());
            telemetry.addData("encoder-back-end", rightDrive.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

}