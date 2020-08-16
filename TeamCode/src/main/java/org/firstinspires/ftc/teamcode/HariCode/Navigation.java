package org.firstinspires.ftc.teamcode.HariCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Navigation", group ="Navigator")
//@Disabled
public class Navigation extends LinearOpMode {

    DcMotor leftDrive, rightDrive;
    int row, column, Xdes, Ydes, Xpar, Ypar, ParValue;



    @Override
    public void runOpMode() throws InterruptedException
    {
      leftDrive = hardwareMap.dcMotor.get("left_Drive");
      rightDrive = hardwareMap.dcMotor.get("right_Drive");

      rightDrive.setDirection(DcMotor.Direction.REVERSE);

      int[][] field = new int[4][5];

      field[Xdes][Ydes] = 0;
      Xpar = Xdes;
      Ypar = Ydes;
      ParValue = field[Xpar][Ypar];

      if ((Math.abs( Xpar))<47){
          field[Xpar+1][Ypar] = ParValue + 1;
          field[Xpar-1][Ypar] = ParValue + 1;
      }
      if ((Math.abs( Ypar))<47){
            field[Xpar][Ypar+1] = ParValue + 1;
            field[Xpar][Ypar-1] = ParValue + 1;
      }









    }
}