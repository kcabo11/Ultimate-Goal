/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 * 
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: HuskyLens_Pixel", group = "Sensor")
//@Disabled
@Config
public class SensorHuskyLens_Pixel extends LinearOpMode {

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

//    private DcMotor leftDrive = null;
//    private DcMotor rightDrive = null;
//    private IMU imu = null;

    public static double Kp = .0005;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double MAX_TURN_SPD = .167;
    public static double MIN_TURN_SPD = -.167;
    double offset = 160; // this is the difference between process variable and setpoint
    double Tp = 50;
    double integral = 0; // the place where we will story our integral
    double lastError = 0; // the place where we will store the last error value
    double derivative = 0; // the place where we will store the derivative
    double v1 = 0;
    double xvalue, error, Turn, frontRight, frontLeft, backRight, backLeft;

    boolean isColor1 = true;
    boolean isColor3 = true;
//    boolean isQrcode2 = true;

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.2;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.


    @Override
    public void runOpMode() {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        // Initialize the drive system variables.
//        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
//        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        imu.resetYaw();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */

        telemetry.update();
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while (opModeIsActive()) {
          //      (leftDrive.isBusy() && rightDrive.isBusy())) {
            if (!rateLimit.hasExpired()) {
            //    continue;
            }
            rateLimit.reset();

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            telemetry.addData("Blocks", blocks);
            isColor1 = false;
            isColor3 = false;
            xvalue = 300;
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());

                if (blocks[i].id == 1) {
                    xvalue = blocks[i].x;
                    isColor1 = true;

                }


//                if (blocks[i].id == 2) {
//                    isQrcode2 = true;
//                }

                if (blocks[i].id == 3) {
                    isColor3 = true;
                }
            }

//            if ((isQrcode2) && (!isQrcode1)) {
//                error = -75;
//            }

            //else
//            if ((isColor3) &&
            if (!isColor1) {
                error = -100;
            }
            else {
                error = xvalue - offset;
            }


            integral = integral + error;
            derivative = error - lastError;
            Turn = Kp * error + Ki * integral + Kd * derivative;
            frontRight = Tp + Turn;
            frontLeft = Tp + Turn;
            backRight = Tp - Turn;
            backLeft = Tp - Turn;
            lastError = error;

            if (Turn > MAX_TURN_SPD) {
                Turn = MAX_TURN_SPD;
            } else if (Turn < -MIN_TURN_SPD) {
                Turn = MIN_TURN_SPD;
            }

            //


//            if (isQrcode3 == true) {
//                v1 = -1 * Turn;
//            }

            v1 = 1 * Turn;


            leftFront.setPower(-v1);
            rightFront.setPower(v1);
            leftBack.setPower(-v1);
            rightBack.setPower(v1);


            if (isColor1 == true) {
                isColor1 = offset == 160;

                leftFront.setPower(0 * -v1);
                rightFront.setPower(0 * v1);
                leftBack.setPower(0 * -v1);
                rightBack.setPower(0 * v1);

            }

//             Step through each leg of the path,
            if (isColor1 == true) {
                driveStraight(DRIVE_SPEED, 24.0, 0.05);    // Drive Forward 24"
                turnToHeading(TURN_SPEED, -45.0);               // Turn  CW to -45 Degrees
                holdHeading(TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for a 1/2 second

                driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
                turnToHeading(TURN_SPEED, 45.0);               // Turn  CCW  to  45 Degrees
                holdHeading(TURN_SPEED, 45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second

                driveStraight(DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
                turnToHeading(TURN_SPEED, 0.0);               // Turn  CW  to 0 Degrees
                holdHeading(TURN_SPEED, 0.0, 1.0);    // Hold  0 Deg heading for 1 second

                driveStraight(DRIVE_SPEED, -48.0, 0.0);    // Drive in Reverse 48" (should return to approx. staring position)
            }


            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.

            telemetry.addData("speed", v1);
            telemetry.addData("error", error);
            telemetry.update();
        }
    }


    private void driveStraight(double driveSpeed, double v, double v1) {

    }

    private void turnToHeading(double turnSpeed, double v) {
    }

    private void holdHeading(double turnSpeed, double v, double v1) {
    }
}



    /*          *** PSEUDOCODE FOR DATA RETRIEVED FROM HUSKYLENS ***

                Assuming we get an x coordinate from the AprilTag we scan,
                how do we center the AprilTag in the vision field of the HuskyLens sensor (effectively turning the robot)?

                1. If x coordinate > origin (160,120), turn right until x coordinate equals origin;
                   If x coordinate < origin (160,120), turn left until x coordinate equals origin

                   *THIS IS DEPENDENT UPON WHAT POSITION IS NEEDED FOR PIXEL PLACEMENT*  - also the Huskylens reads distance
                2. If y coordinate > origin (160,120), move forward until y coordinate equals origin;
                   If y coordinate < origin (160,120), move backward until y coordinate equals origin


                From here, we can move towards the backdrop, and place a pixel(s)
                1. Move forward until you get to proper location for pixel release
                2. Release pixel(s) onto backdrop

                *** PID PSEUDOCODE ***

                1. set movement motors to (motors)
                2. set (Integral) to 0
                3. set (lastError) to 0
                -- FOREVER LOOP --
                4. set (Error) to [(sensor) - 50]
                5. set (P-fix) to [Error * 0.3]
                6. set (Integral) to [Integral + Error]
                7. set (I-fix) to [Integral * 0.001]
                8. set (Derivative) to [Error - lastError]
                9. set (lastError) to [Error]
                10. set (D-fix) to [Derivative * 1]
                11. set (correction) to [P-fix + I-fix + D-fix]
                12. start moving at [40 + correction][40 - correction] %power
                -- FOREVER LOOP --
    */

// ============================================================================================================================

/*              *** PSEUDOCODE FOR THURSDAY TASK***

                1. See the position/location of team marker
                2. Use gyro turn (code in "RobotAutoDriveByGyro_Linear.java"), and face center towards the marker
                3. Drive towards the marker (using gyro drive from the example code)

                Steps:
                1. Configure huskylens to see temporary team marker
                2. Examine example gyro code


                From here, you can write pseudocode for grabbing the marker and placing it
                4.

 */