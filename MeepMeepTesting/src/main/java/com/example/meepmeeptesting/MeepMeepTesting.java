package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.entity.TrajectorySequenceEntity;

//public class RoadRunnerCenterStage {
//}

//    The code snippet and explanation within the video is currently not up-to-date with the latest MeepMeep 2.0.x API


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(60, 60,Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, 1.55))

//                      *** To run a specific position, comment out the other code (this will be modified later) ***

//                      // CENTER POSITION
//                                .forward(30)
//                                .back(2)
//
//                                // sleep
//                                .splineToSplineHeading(new Pose2d(45, -35, Math.toRadians(180)), Math.toRadians(0))

                        // LEFT POSITION
//                                .strafeRight(10)
//                                .forward(5)
//                                .splineToSplineHeading(new Pose2d(25, -35, Math.toRadians(180)), Math.toRadians(0))
//                                .forward(13)
//
//                                // sleep
//                                .back(34)

                        // RIGHT POSITION
//                                .forward(30)
//                                .strafeLeft(2)
//                                .turn(Math.toRadians(-90))
//                                .forward(3)
//                                .back(2)
//                                .strafeRight(23)
//                                .forward(20)
//                                .splineToSplineHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.toRadians(0))


//.strafeLeft(22)
//                                .forward(47)
//                                .turn(Math.toRadians(90))
//                                .forward(3)
//                                .back(5)
//                                .turn(Math.toRadians(180))
//                                .forward(115)
                                .forward(25)
                                .back(28)
                                .strafeRight(75)
                                .forward(26.5)
                                .turn(Math.toRadians(90))
                                .back(8)
                                .strafeRight(24)
                                .back(10)


                                .build()
                );





        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

        // Robot moves to the specified coordinates in a spline path
// while separately linearly interpolating the heading
//
// The heading interpolates to the heading specified in `endPose`.
// Setting `endTangent` affects the shape of the spline path itself.
//
// Due to the holonomic nature of mecanum drives, the bot is able
// to make such a movement while independently controlling heading.

// 🚨  Will cause PathContinuityException's!! 🚨
// Use splineToSplineHeading() if you are chaining these calls

    }
}