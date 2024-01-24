/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack     = null;
    public DcMotor  rightBack   = null;


    public DcMotor  liftMotor     = null;
    public DcMotor  liftDownMotor     = null;
    public DcMotor  intakeMotor     = null;
    public DcMotor  pixelLiftMotor     = null;
    public Servo    linearSlide    = null;
    public Servo    pixelPlacerServo = null;
    //public Servo    rightClaw   = null;

    // ============= POSSIBLE SERVOS: ========
    public Servo leftPixelLatch = null;
    public Servo rightPixelLatch = null;
    // =======================================
    public Servo pixelPlacer = null;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;


    double clawOffset = 0;
    double scaleTurningSpeed = .8;
    double scaleFactor = .8;
    int direction = -1;

    //  WHERE WOULD INTAKE BE PLACED HERE IN THE INITIALIZATION??
    // Initialize the following:
    // Linear slide ~~ servo
    // Intake (left and right) ~~ servo
    // Pixel placer ~~ servo
    // Left pixel latch ~~ possible servo
    // Right pixel latch ~~ possible servo


    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftDownMotor    = hardwareMap.get(DcMotor.class, "liftDownMotor");
        liftMotor    = hardwareMap.get(DcMotor.class, "liftMotor");
        pixelLiftMotor    = hardwareMap.get(DcMotor.class, "pixelLiftMotor");
        pixelPlacerServo = hardwareMap.get(Servo.class, "pixelPlacerServo");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // NO NEED FOR ENCODERS!!!
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
//        linearSlide = hardwareMap.get(Servo.class, "linearSlide");
//        pixelPlacer = hardwareMap.get(Servo.class, "pixelPlacer");
//
//        // Possible Servos:
//        leftPixelLatch = hardwareMap.get(Servo.class, "leftPixelLatch");
//        rightPixelLatch = hardwareMap.get(Servo.class, "rightPixelLatch");


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ========================== DRIVE CONTROLLER ================================================

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            // When the direction value is reversed this if statement inverts the addition and subtraction for turning.

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            // Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v3);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {
                final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }

            if (gamepad2.dpad_up) { //checks out
                pixelLiftMotor.setPower(-.5);
            } else if (gamepad2.dpad_down) {
                pixelLiftMotor.setPower(.2);
            } else
                pixelLiftMotor.setPower(0);

            if (gamepad2.a)
                liftMotor.setPower(1);
            else
                liftMotor.setPower(0);
            // This is the pixelPlacerServo
            if (gamepad2.right_bumper) {
                pixelPlacerServo.setPosition(1);
            } else if (gamepad2.left_bumper) {
                pixelPlacerServo.setPosition(0);
            }

            if (gamepad2.y) {
                liftDownMotor.setPower(1);
                liftMotor.setPower(-.5);
            } else
                liftDownMotor.setPower(0);

            //Intake out

            if (gamepad2.b) {
                //intakeMotor.setPower(.5);
                intakeLeft.setPower(1);
                intakeRight.setPower(-1);
            }
            //Intake in
            else if (gamepad2.x) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(1);
                //intakeMotor.setPower(-.5);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                //intakeMotor.setPower(0);
            }


            //moving frontright changes the front left value
            //front left changes the right front value
            //left back changes left back
            //right back should change the right back

//            if (gamepad1.y) // DOUBLE TAPPED!!
            // shoot airplane


            // ========================== OPERATOR CONTROLLER ===========================================

//            intakeLeft.setPower(1);
//            intakeRight.setPower(-1);

            //D pad or bumper: up/down for linear slide

            //Use gamepad left & right triggers to manage intake and outake
            if (gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0) {
                liftDownMotor.setPower(-1);
            }

                //intakeRight
//
//            else if (gamepad2.left_trigger)
//                //clawOffset -= CLAW_SPEED;
//                //intakeLeft

            // Move both servos to new position.  Assume servos are mirror image of each other.
            //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            leftClaw.setPosition(MID_SERVO + clawOffset);
//            rightClaw.setPosition(MID_SERVO - clawOffset);

//            // Use gamepad button (A) to bring up main plate
//            if (gamepad2.a)
//                //pixelPlacer.setPosition();
//                // release pixels
//                else if (gamepad2.x)
//                    //open left pixel latch
//                    else if (gamepad2.b)
                        //open right pixel latch
//            else
//                leftArm.setPower(0.0);




            // ==================================== TELEMETRY =========================================
            // Send telemetry message to signify robot running;

            telemetry.addData("leftFront: ", leftFront.getCurrentPosition());
            telemetry.addData("leftBack: ", leftBack.getCurrentPosition());
            telemetry.addData("rightBack: ", rightBack.getCurrentPosition());
            telemetry.addData("rightFront: ", rightFront.getCurrentPosition());
            telemetry.addData("pixelLiftMotor pos: ", pixelLiftMotor.getCurrentPosition());
            telemetry.addData("pixelLiftMotor pwr: ", pixelLiftMotor.getPower());
            telemetry.addData("liftMotor pwr: ", liftMotor.getPower());
            telemetry.addData("DPAD UP", gamepad2.dpad_up);
            telemetry.addData("DPAD DOWN", gamepad2.dpad_down);
            telemetry.addData("gamepad2.b", gamepad2.b);
            telemetry.addData("gamepad2.x", gamepad2.x);
            telemetry.addData("gamepad2.left_bumper", gamepad2.left_bumper);
            telemetry.addData("gamepad2.right_bumper", gamepad2.right_bumper);

            telemetry.update();

            //

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}


/* PseudoCode TeleOp

1.Drive:
Left joystick tilted north - Forward-All 4 motors turn forward
Left joystick tilted south - Backward-All 4 motors turn backward
Left joystick tilted west - Left
Left joystick tilted east - Right
Left joystick tilted in a diagonal - Strafe

2.Grab Pixels:
-Intake Pixels

3.Transport Pixels:
Left joystick tilted north - Forward-All 4 motors turn forward
Left joystick tilted south - Backward-All 4 motors turn backward
Left joystick tilted west - Left
Left joystick tilted east - Right
Left joystick tilted in a diagonal - Strafe

4.Place Pixels:
-Slide plate using linear slide
-Drop pixels onto backdrop

 */