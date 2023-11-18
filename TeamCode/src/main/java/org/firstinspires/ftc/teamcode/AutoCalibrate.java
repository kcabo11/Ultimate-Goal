package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutoCalibrate")
public class AutoCalibrate extends LinearOpMode {
    //
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    //Calculate encoder conversion
    Integer cpr = 145; //counts per rotation
    Integer gearratio = 2;
    Double diameter = 3.77953;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    Double bias = 0.8;//adjust until your robot goes 20 inches
    //
    Double conversion = cpi * bias;
    //
    public void runOpMode() {
        //
        frontleft = hardwareMap.dcMotor.get("leftFront");
        frontright = hardwareMap.dcMotor.get("rightFront");
        backleft = hardwareMap.dcMotor.get("leftBack");
        backright = hardwareMap.dcMotor.get("rightBack");
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        backright.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left

        telemetry.addData("counts per inch", cpi);
        telemetry.addData("counts per revolution", cpr);
        telemetry.addData("gear ratio", gearratio);
        telemetry.addData("conversion", conversion);

        telemetry.addData("leftFront: ", frontleft.getCurrentPosition());
        telemetry.addData("leftBack: ", backleft.getCurrentPosition());
        telemetry.addData("rightBack: ", backright.getCurrentPosition());
        telemetry.addData("rightFront: ", frontright.getCurrentPosition());
        telemetry.update();

        waitForStartify();
        //
        moveToPosition(6, .05);//Don't change this line, unless you want to calibrate with different speeds
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        //
        if (inches < 5) {
            int move = (int) (Math.round(inches * conversion));
            //
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move);
            backright.setTargetPosition(backright.getCurrentPosition() + move);
            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);
            //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 5) * conversion));
//            int movefl2 = frontleft.getCurrentPosition() + (int) (Math.round(inches * conversion));
//            int movefr2 = frontright.getCurrentPosition() + (int) (Math.round(inches * conversion));
//            int movebl2 = backleft.getCurrentPosition() + (int) (Math.round(inches * conversion));
//            int movebr2 = backright.getCurrentPosition() + (int) (Math.round(inches * conversion));
            //
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + move1);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move1);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move1);
            backright.setTargetPosition(backright.getCurrentPosition() + move1);

            telemetry.addData("fl curr", frontleft.getCurrentPosition());
            telemetry.addData("fr curr", frontright.getCurrentPosition());
            telemetry.addData("bl curr", backleft.getCurrentPosition());
            telemetry.addData("br curr", backright.getCurrentPosition());
            telemetry.addData("fl tgt", frontleft.getTargetPosition());
            telemetry.addData("fr tgt", frontright.getTargetPosition());
            telemetry.addData("bl tgt", backleft.getTargetPosition());
            telemetry.addData("br tgt", backright.getTargetPosition());
            telemetry.update();
            sleep(10000);

            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);

            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            frontright.setPower(0);
            frontleft.setPower(0);
            backright.setPower(0);
            backleft.setPower(0);
            return;
        }

            //
//            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
//            }
//            //
//            frontleft.setTargetPosition(movefl2);
//            frontright.setTargetPosition(movefr2);
//            backleft.setTargetPosition(movebl2);
//            backright.setTargetPosition(movebr2);
//            //
//            frontleft.setPower(.1);
//            frontright.setPower(.1);
//            backleft.setPower(.1);
//            backright.setPower(.1);
//            //
//            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
//            }
//            frontleft.setPower(0);
//            frontright.setPower(0);
//            backleft.setPower(0);
//            backright.setPower(0);
//        }
        return;
    }
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForStartify() {
        waitForStart();
    }
}