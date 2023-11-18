package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous()
public class AutoDriveForward extends LinearOpMode {

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private VisionPortal visionPortal_2;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;

    private IMU imu = null;      // Control/Expansion Hub IMU

    static /* final */ double FORWARD_SPEED = 0.3;
    static /* final */ double REVERSE_SPEED = -0.3;
    static /* final */ double TURN_SPEED = 0.3;
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
    private static int DESIRED_TAG_ID = 1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private double drive = 0;        // Desired forward power/speed (-1 to +1)
    private double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    private double turn = 0;        // Desired turning power/speed (-1 to +1)

    @Override
    public void runOpMode() {

        // Initialize the Apriltag Detection process and Webcam 2

        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        //        visionPortal.resumeStreaming();
//        visionPortal_2.resumeLiveView();

        //initAprilTag();
//        setManualExposure(6, 250);
//        visionPortal_2.stopStreaming();
//        visionPortal_2.stopLiveView();


        telemetry.addData("Identified", visionProcessor.getSelection());
        telemetry.addData("RectLeft Saturation: ", visionProcessor.satRectLeft);
        telemetry.addData("RectMiddle Saturation: ", visionProcessor.satRectMiddle);
        telemetry.addData("Average Saturation: ", visionProcessor.satAverage);

        // Initialize the drive system variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
//            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.addData("middle sat", visionProcessor.satRectMiddle);
            telemetry.addData("left sat", visionProcessor.satRectLeft);
            telemetry.addData("sat delta", visionProcessor.satAverage);
            telemetry.addData("position", visionProcessor.getSelection());
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
        }



        waitForStart();
        leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() <3.5) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
//
//        FirstVisionProcessor.Selected selectedTag = visionProcessor.getSelection();
//
//        // =============================================== MIDDLE POSITION =================================================================
//        if  (selectedTag == FirstVisionProcessor.Selected.MIDDLE) {
//            // ASSUMING SPIKE MARK IS MIDDLE POSITION
//            // Step 1: Deposit pixel (push pixel to MIDDLE spike mark)
//
//            // Step 1.a:  Drive forward for __ seconds
//            leftFront.setPower(-FORWARD_SPEED);
//            rightFront.setPower(-FORWARD_SPEED);
//            rightBack.setPower(-FORWARD_SPEED);
//            leftBack.setPower(-FORWARD_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds() < 1.8) {
//                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//            leftBack.setPower(0);
//            intakeLeft.setPower(.5);
//            intakeRight.setPower(-.5);
//            }
//            sleep(1000);
//
//            // Step 2: Turn LEFT 90 degrees
//            //Spin left for 1.3 seconds
//            //leftFront.setPower(TURN_SPEED);
////            rightFront.setPower(-TURN_SPEED);
////            rightBack.setPower(-TURN_SPEED);
////            leftBack.setPower(TURN_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//
//            // Step 4: Use the automatic code to drive to apriltag
//            DESIRED_TAG_ID = 2;
//            //driveToAprilTag();
//
//        // =============================================== LEFT POSITION =================================================================
//        else if (selectedTag == FirstVisionProcessor.Selected.LEFT) {
//            // ASSUMING SPIKE MARK IS LEFT POSITION
//            // Step 1: Deposit pixel (push pixel to LEFT spike mark)
//
//            // Step 1.a:  Drive forward for __ seconds
//            // Decrease Speed first
//            FORWARD_SPEED = 0.25;
//            TURN_SPEED = 0.25;
//
//            leftFront.setPower(-FORWARD_SPEED);
//            rightFront.setPower(-FORWARD_SPEED);
//            rightBack.setPower(FORWARD_SPEED);
//            leftBack.setPower(FORWARD_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds() < 1.4) {
//                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//            leftBack.setPower(0);
//
//            sleep(1000);
//
//            // Step 2: Turn LEFT 90 degrees
//            //Spin left for __ seconds
//            leftFront.setPower(TURN_SPEED);
//            rightFront.setPower(-TURN_SPEED);
//            rightBack.setPower(-TURN_SPEED);
//            leftBack.setPower(TURN_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < 2.9)) {
//                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//
//            // Return speed:
//            FORWARD_SPEED = 0.3;
//            TURN_SPEED = 0.3;
//            // Step 3: Place pixel
//            leftFront.setPower(-FORWARD_SPEED);
//            rightFront.setPower(-FORWARD_SPEED);
//            rightBack.setPower(FORWARD_SPEED);
//            leftBack.setPower(FORWARD_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds() < .4) {
//                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//            leftBack.setPower(0);
//
//            sleep(1000);
//
//            // Step 4: Use the automatic code to drive the qr code
//            DESIRED_TAG_ID = 1;
//            //driveToAprilTag();
//        }
//        // =============================================== RIGHT POSITION =================================================================
//        else if (selectedTag == FirstVisionProcessor.Selected.RIGHT) {
//            // ASSUMING SPIKE MARK IS RIGHT POSITION
//            // Step 1: Deposit pixel (push pixel to RIGHT spike mark)
//
//
//            // Step 1.a:  Slighty turn right
//            //Spin right for __ seconds
//            leftFront.setPower(-TURN_SPEED);
//            rightFront.setPower(TURN_SPEED);
//            rightBack.setPower(TURN_SPEED);
//            leftBack.setPower(-TURN_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < .6)) {
//                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//
//            // Step 1.a:  Drive forward for __ seconds, and place pixel
//            leftFront.setPower(-FORWARD_SPEED);
//            rightFront.setPower(-FORWARD_SPEED);
//            rightBack.setPower(FORWARD_SPEED);
//            leftBack.setPower(FORWARD_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds() < 1) {
//                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//            leftBack.setPower(0);
//
//            sleep(1000);
//
//            // Step 1.c:  Scoot back (so you dont hit the spike marker)
//            leftFront.setPower(-REVERSE_SPEED);
//            rightFront.setPower(-REVERSE_SPEED);
//            rightBack.setPower(REVERSE_SPEED);
//            leftBack.setPower(REVERSE_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && runtime.seconds() < .5) {
//                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//            leftBack.setPower(0);
//
//            sleep(1000);
//
//
//            // Step 2: Turn left 120 degrees
//            //Spin left for __ seconds
//            leftFront.setPower(TURN_SPEED);
//            rightFront.setPower(-TURN_SPEED);
//            rightBack.setPower(-TURN_SPEED);
//            leftBack.setPower(TURN_SPEED);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < 2.3)) {
//                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//
//            // Step 4: Use the automatic code to drive to apriltag
//            DESIRED_TAG_ID = 3;
//            //driveToAprilTag();
//        }
//        // TODO: Strafe right, drive forward to park in backstage
        //!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal_2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal_2 == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal_2.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal_2.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal_2.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal_2.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    private void driveToAprilTag() {

        targetFound = false;
        desiredTag = null;
        double rangeError;
        double headingError;
        double yawError;

        visionPortal.stopStreaming();
        visionPortal_2.resumeStreaming();

        while (opModeIsActive()) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        telemetry.addData("target found", DESIRED_TAG_ID);

                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
//
//            // Tell the driver what we see, and what to do.
//            if (targetFound) {
//                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
//                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
//                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
//            } else {
//                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
//            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            //            if (gamepad1.left_bumper && targetFound) {
            if (targetFound) { //(targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                headingError = desiredTag.ftcPose.bearing;
                yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            } else {
//
//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
//                strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
//                turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);

        }
    }
}