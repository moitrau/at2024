package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueAlliance2BlueSample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-9, 64, Math.toRadians(90)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-9,38,Math.toRadians(90)))
                        .setReversed(false)

                        .splineToLinearHeading(new Pose2d(-34,34,Math.toRadians(-90)),Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-45,12,Math.toRadians(-90)),Math.toRadians(145))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-45,56,Math.toRadians(-90)))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-54,12,Math.toRadians(-90)),Math.toRadians(145))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-54,56,Math.toRadians(-90)))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-7,38,Math.toRadians(90)),Math.toRadians(-60))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-54,60,Math.toRadians(-90)),Math.toRadians(90))

                        //.lineToLinearHeading(new Pose2d(-58,46,Math.toRadians(-90)))
                        //.lineToLinearHeading(new Pose2d(-58,56,Math.toRadians(-90)))
                        //.splineToLinearHeading(new Pose2d(-7,38,Math.toRadians(90)),Math.toRadians(-90))
                        //.splineToLinearHeading(new Pose2d(-52,56,Math.toRadians(-90)),Math.toRadians(90))

                        /*.setReversed(true)
                        .splineToLinearHeading(new Pose2d(54,54,Math.toRadians(-135)),Math.toRadians(90))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(58,46,Math.toRadians(-90)),Math.toRadians(90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(54,54,Math.toRadians(-135)),Math.toRadians(90))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(59,14,Math.toRadians(-90)),Math.toRadians(60))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(59,54,Math.toRadians(-90)))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(28,12,Math.toRadians(-180)),Math.toRadians(150))*/
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}