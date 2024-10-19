package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "BasicAuton")
public class BasicAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d startPose;
        startPose = new Pose2d(-12, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                //.forward(16)
                //.turn(Math.toRadians(90))
                //.forward(55)
                //.turn(Math.toRadians(45))
                //.forward(10)
                .splineToLinearHeading(new Pose2d(50,55,Math.toRadians(45)),Math.toRadians(90))
                .build();

        drive.followTrajectorySequence(traj);



    }
}
