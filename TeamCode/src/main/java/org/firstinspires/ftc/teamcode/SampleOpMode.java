package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@TeleOp
@Disabled
//sample opmode
public class SampleOpMode extends OpMode {
    private DcMotorController dcMC;
    private DcMotor testMotor;
    private int count =0;
    @Override
    public void init() {
        //dcMC = hardwareMap.get("drive_controller");

        testMotor = (DcMotor) hardwareMap.get("testMotor");
      telemetry.addData("Status", "Initialized");
      if(testMotor != null)
      {
          telemetry.addData("Status","Motor Valid");
      }
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)


    }

    @Override
    public void loop() {

        telemetry.addData("Status", "In test "+count++);
        telemetry.update();
        if(testMotor!=null)
        {
            int nMotorPort = testMotor.getPortNumber();
            telemetry.addData("Status","Port Number:"+nMotorPort);
        }

    }
}
