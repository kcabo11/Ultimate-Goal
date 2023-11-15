package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous()
public class VisionProcessorTestFromBook extends LinearOpMode {

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private IMU imu = null;      // Control/Expansion Hub IMU

    static final double FORWARD_SPEED = 0.2;
    static final double TURN_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime();


    // ==================================== Auto Drive to April Tag Omni Initialization =================================================
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override
    public void runOpMode() {

        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);

        telemetry.addData("Identified", visionProcessor.getSelection());
        telemetry.addData("RectLeft Saturation: ", visionProcessor.satRectLeft);
        telemetry.addData("RectMiddle Saturation: ", visionProcessor.satRectMiddle);
        telemetry.addData("Average Saturation: ", visionProcessor.satAverage);

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)


        // Initialize the drive system variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftFront.setDirection(DcMotor.Direction.FORWARD);
//        leftBack.setDirection(DcMotor.Direction.FORWARD);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        // NOTE: we do plan to use timed turning for Nov 18 tournament
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // =========================================================================================
        // Wait for the game to start (Display Gyro value while waiting)
//        while (opModeInInit()) {
//            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
//            telemetry.update();
//        }
//                ============================== ORRRRR ======================================
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        // =========================================================================================


        if (visionProcessor.getSelection() == FirstVisionProcessor.Selected.NONE) {
            // ASSUMING SPIKE MARK IS MIDDLE POSITION
            // ==================================== Step 1: Drive forward for 2 seconds ====================================

            // **** TIME DRIVING INSERTED IN STEP 2 ****

            // ==================================== Step 2: Deposit pixel (push pixel to MIDDLE spike mark) =============================

            // ****Insert time driving here****
//            leftFront.setDirection(DcMotor.Direction.FORWARD);
//            rightFront.setDirection(DcMotor.Direction.FORWARD);
//            leftBack.setDirection(DcMotor.Direction.FORWARD);
//            rightBack.setDirection(DcMotor.Direction.FORWARD);

            // Step 1:  Drive forward for 3 seconds
            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            rightBack.setPower(FORWARD_SPEED);
            leftBack.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
//
//            // Step 3:  Drive Backward for 1 Second
//            leftDrive.setPower(-FORWARD_SPEED);
//            rightDrive.setPower(-FORWARD_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//
//            // Step 4:  Stop
//            leftDrive.setPower(0);
//            rightDrive.setPower(0);
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//            sleep(1000);


            // ==================================== Step 3: Turn right 90 degrees by gyro ====================================

            //Spin left for 1.3 seconds
            leftFront.setPower(TURN_SPEED);
            rightFront.setPower(-TURN_SPEED);
            rightBack.setPower(-TURN_SPEED);
            leftBack.setPower(TURN_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .45)) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // ==================================== Step 4: Use the automatic code to drive the qr code ======================
            //** Insert omni wheel logitech camera code here **
            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
//            if (gamepad1.left_bumper && targetFound) {
//            if (targetFound) {
//
//                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//                double  headingError    = desiredTag.ftcPose.bearing;
//                double  yawError        = desiredTag.ftcPose.yaw;
//
//                // Use the speed and turn "gains" to calculate how we want the robot to move.
//                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            } else {
//
//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            }
//            telemetry.update();
//
//            // Apply desired axes motions to the drivetrain.
//            moveRobot(drive, strafe, turn);
//            sleep(10);
//        }

        }
//}

////    public double getHeading() {
////        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
////        return orientation.getYaw(AngleUnit.DEGREES);
////    }
//
//    public void moveRobot(double x, double y, double yaw) {
//        // Calculate wheel powers.
//        double leftFrontPower    =  x -y -yaw;
//        double rightFrontPower   =  x +y +yaw;
//        double leftBackPower     =  x +y -yaw;
//        double rightBackPower    =  x -y +yaw;
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send powers to the wheels.
//        leftFront.setPower(leftFrontPower);
//        rightFront.setPower(rightFrontPower);
//        leftBack.setPower(leftBackPower);
//        rightBack.setPower(rightBackPower);
//    }
    }
}