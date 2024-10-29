package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "01_BasicAutonBlueAllianceYellowSample")
public class BasicAutonBAYS extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d startPose;
        startPose = new Pose2d(9, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(9,37,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(37,37,Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(54,54,Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(58,40,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(54,54,Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(58,40,Math.toRadians(-45)))

                .lineToLinearHeading(new Pose2d(54,54,Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(27,12,Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(32,12,Math.toRadians(-0)),Math.toRadians(-160))
                .build();

        drive.followTrajectorySequence(traj);

    }

}

