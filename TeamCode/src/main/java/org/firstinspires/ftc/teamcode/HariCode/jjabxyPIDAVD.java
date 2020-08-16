package org.firstinspires.ftc.teamcode.HariCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="jjabxyPIDAVD", group="Driver")
//@Disabled
public class jjabxyPIDAVD extends LinearOpMode
{
    DcMotor                 leftDrive, rightDrive, leftLift, rightLift;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
    boolean                 xButton, yButton;
    PIDController           pidRotate, pidDrive;
    Servo leftBar, rightBar;
    float   leftPower, rightPower, xValue, yValue, liftY;
    double leftBarPos, rightBarPos;
    double MIN_POSITION = 0, MAX_POSITION = 1;



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
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftBarPos = 0.5;
        rightBarPos = 0.5;



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.

        pidRotate = new PIDController(.003, .00003, 0);


        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // drive until end of period.

        while (opModeIsActive())
        {
            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.update();

            if (xButton||yButton){
                leftDrive.setPower(power - correction);
                rightDrive.setPower(power + correction);
            }else{
                leftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
                rightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
            }

            leftLift.setPower(0.2 * Range.clip(liftY, -1.0, 1.0));
            rightLift.setPower(0.2* Range.clip(liftY, -1.0, 1.0));
            leftBar.setPosition(Range.clip(leftBarPos, MIN_POSITION, MAX_POSITION));
            rightBar.setPosition(Range.clip(rightBarPos, MIN_POSITION, MAX_POSITION));



            yValue = gamepad1.left_stick_y * -1;
            xValue = gamepad1.left_stick_x * -1;
            liftY = gamepad1.right_stick_y* -1;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            if (gamepad1.a && leftBarPos > MIN_POSITION) leftBarPos -= .01;
            if (gamepad1.a && rightBarPos < MAX_POSITION) rightBarPos += .01;

            if (gamepad1.b && leftBarPos < MAX_POSITION) leftBarPos += .01;
            if (gamepad1.b && rightBarPos > MIN_POSITION) rightBarPos -= .01;

            if(yValue<0.4)
                if(-0.4<yValue){
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                }

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            xButton = gamepad1.x;
            yButton = gamepad1.y;

            if ( xButton || yButton)
            {
                // backup.
                leftDrive.setPower(-power);
                rightDrive.setPower(-power);

                sleep(500);

                // stop.
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                // turn 90 degrees right.
                if ( xButton) rotate(-90, power);

                // turn 90 degrees left.
                if (yButton) rotate(90, power);
            }
        }

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                leftDrive.setPower(power);
                rightDrive.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftDrive.setPower(-power);
                rightDrive.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftDrive.setPower(-power);
                rightDrive.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}