package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AkshayanSquare", group = "Square")
//@Disabled
public class AkshayanSquare extends LinearOpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(500);        // wait so that above telemetry is visible.

        // each iteration of this for loop will drive one side of the square.

        for(int i = 0; i < 4; i++)
        {
            telemetry.addData("Mode", "driving side " + (i + 1));
            telemetry.update();

            leftDrive.setPower(0.25);
            rightDrive.setPower(0.25);

            sleep(1000); // drive straight for 1 second.

            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);

            sleep(500);  // wait half second for bot to stop moving.

            // now set motors, one forward one reverse. Should cause the bot to rotate.

            leftDrive.setPower(0.25);
            rightDrive.setPower(-0.25);

            sleep(1700); // adjust this delay to get the bot to rotate 90 degrees.

            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);

            sleep(500); // wait for bot to stop moving.
        }

        // make sure the motors are off.

        rightDrive.setPower(0);
        leftDrive.setPower(0);
    }

}
