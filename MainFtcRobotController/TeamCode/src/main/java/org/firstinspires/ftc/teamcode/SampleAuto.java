package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Simple Auto", group = "Autonomous")
public class SampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Define the starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Create a trajectory to drive forward
        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Follow the trajectory
        drive.followTrajectory(trajectory);
    }
}
