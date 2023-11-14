package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class VisionPortalStreamingOpMode extends LinearOpMode {
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private int width;
        enum SpikeLocation {
            LEFT,
            RIGHT,
            NONE
        }

        SpikeLocation location;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // "Mat" stands for matrix, which is basically the image that the detector will process
            // the input matrix is the image coming from the camera
            // the function will return a matrix to be drawn on your phone's screen

            // The detector detects regular stones. The camera fits two stones.
            // If it finds one regular stone then the other must be the skystone.
            // If both are regular stones, it returns NONE to tell the robot to keep looking

            // Make a working copy of the input matrix in HSV
            Mat mat = new Mat();
            Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

            // if something is wrong, we assume there's no skystone
            if (mat.empty()) {
                location = SpikeLocation.NONE;
                return frame;
            }

            // We create a HSV range for yellow to detect regular stones
            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value
            Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
            Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
            Mat thresh = new Mat();

            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(mat, lowHSV, highHSV, thresh);

            // Use Canny Edge Detection to find edges
            // you might have to tune the thresholds for hysteresis
            Mat edges = new Mat();
            Imgproc.Canny(thresh, edges, 100, 300);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            // We then find the bounding rectangles of those contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }



            // Iterate and check whether the bounding boxes
            // cover left and/or right side of the image
            double left_x = 0.25 * width;
            double right_x = 0.75 * width;
            boolean left = false; // true if regular stone found on the left side
            boolean right = false; // "" "" on the right side
            for (int i = 0; i != boundRect.length; i++) {
                if (boundRect[i].x < left_x)
                    left = true;
                if (boundRect[i].x + boundRect[i].width > right_x)
                    right = true;

                // draw red bounding rectangles on mat
                // the mat has been converted to HSV so we need to use HSV as well
                Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
            }

            // if there is no yellow regions on a side
            // that side should be a Skystone
            if (!left) location = SpikeLocation.LEFT;
            else if (!right) location = SpikeLocation.RIGHT;
                // if both are true, then there's no Skystone in front.
                // since our team's camera can only detect two at a time
                // we will need to scan the next 2 stones
            else location = SpikeLocation.NONE;

            return mat; // return the mat with rectangles drawn
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        public SpikeLocation getLocation() {
            return this.location;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
            .addProcessor(processor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);

        waitForStart();

        while (opModeIsActive()) {
            CameraStreamProcessor.SpikeLocation location = processor.getLocation();

            telemetry.addData("location", location);
            telemetry.update();

            sleep(100L);
        }
    }
}
