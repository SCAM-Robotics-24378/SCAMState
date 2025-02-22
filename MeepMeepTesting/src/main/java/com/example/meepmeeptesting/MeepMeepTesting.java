package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Pose2d bucketScorePose = new Pose2d(-53,-53, Math.toRadians(45));
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity bucketBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                        new Pose2d(-23.75 + (15.5 / 2), (-70.3 + (15.5 / 2)), Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(-11, -35, Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(-34, -45, Math.toRadians(125)))
                        .lineToSplineHeading(bucketScorePose)
                        .lineToSplineHeading(new Pose2d(-47, -45, Math.toRadians(115)))
                        .lineToSplineHeading(bucketScorePose)
                        .lineToSplineHeading(new Pose2d(-51, -38, Math.toRadians(145)))
                        .lineToSplineHeading(bucketScorePose)
                        .lineToSplineHeading(new Pose2d(-40, -10, 0))
                        .lineToSplineHeading(new Pose2d(-27.5, -10, 0))
                        .lineToSplineHeading(new Pose2d(-40, -10, 0))
                        .lineToSplineHeading(bucketScorePose)
                        .lineToSplineHeading(new Pose2d(-40, -10, 0))
                        .lineToSplineHeading(new Pose2d(-27.5, -10, 0))
                        .build());

        RoadRunnerBotEntity specimenBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                        new Pose2d(23.75 - (15.5 / 2), (-70.3 + (15.5 / 2)), Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(11, -35, Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(34, -42, Math.toRadians(55)))
                        .turn(Math.toRadians(-95))
                        .lineToSplineHeading(new Pose2d(44, -42, Math.toRadians(55)))
                        .turn(Math.toRadians(-95))
                        .lineToSplineHeading(new Pose2d(54, -42, Math.toRadians(55)))
                        .lineToSplineHeading(new Pose2d(48, -53, Math.toRadians(-45)))
                        .turn(Math.toRadians(135))
                        .lineToSplineHeading(new Pose2d(35, -58, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(11, -35, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(35, -58, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(11, -35, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(35, -58, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(11, -35, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(35, -58, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(11, -35, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(35, -58, Math.toRadians(90)))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bucketBot)
                .addEntity(specimenBot)
                .start();
    }
}