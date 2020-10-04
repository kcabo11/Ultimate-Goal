package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.examples.LibraryOpenCV;

@Config
@Autonomous(group = "Blue Side Skystone Auto")
public class TestingOpenCV extends LinearOpMode {
    static HardwareBeep robot;
    LibraryOpenCV opencv;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareBeep();
        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        waitForStart();
        opencv = new LibraryOpenCV(robot,telemetry, hardwareMap);
        opencv.initOpenCV();
        opencv.findRingConfig();
        sleep(5000);
        telemetry.addData("OpenCV initialized", "");
        telemetry.update();
    }
}
