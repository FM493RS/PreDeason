package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// below is the Annotation that registers   this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name = "Partial Square", group = "Square")
//@Disabled
public class LavanyaPartialSquare extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power.

        //leftMotor.setPower(0.25);
        //rightMotor.setPower(0.25);

        sleep(500);        // wait for 0.5 seconds.

        // each iteration of this for loop will drive one side of the square.

        for(int i = 0; i < 4; i++)
        {
            telemetry.addData("Mode", "driving side " + (i + 1));
            telemetry.update();

            leftMotor.setPower(0.25);
            rightMotor.setPower(0.25);

            sleep(1000); // drive straight for 1 second.

            leftMotor.setPower(-0.75);
            rightMotor.setPower(0.75);

            sleep(600);  // wait half second for bot to stop moving.

        }

        // set motor power to zero to stop motors.

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}