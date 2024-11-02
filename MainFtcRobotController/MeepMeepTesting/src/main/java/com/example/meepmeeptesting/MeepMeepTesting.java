package com.example.meepmeeptesting;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-32.5, -64, Math.toRadians(-90)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-54,-54,Math.toRadians(45)),Math.toRadians(-90))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-50,-46,Math.toRadians(90)),Math.toRadians(-90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-54,-54,Math.toRadians(45)),Math.toRadians(-90))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-58,-46,Math.toRadians(90)),Math.toRadians(-90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-54,-54,Math.toRadians(45)),Math.toRadians(-90))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-59,-14,Math.toRadians(90)),Math.toRadians(120))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-59,-54,Math.toRadians(90)))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-28,-10,Math.toRadians(0)),Math.toRadians(10))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}