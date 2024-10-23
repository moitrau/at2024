package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="ChassisRunnerv2")
public class ChassisRunner extends LinearOpMode {

    private DcMotor rightRear;
    private DcMotor rightFront;

    private DcMotor leftRear;
    private DcMotor leftFront;
    /**
     * Initializes a new stopwatch.
     */

    @Override
    public void runOpMode() throws InterruptedException {


        rightRear = hardwareMap.get(DcMotor.class, "rearRight");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "rearLeft");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if(gamepad1.dpad_up){drive.setWeightedDrivePower(new Pose2d(0.2,0,0));}
            if(gamepad1.dpad_down){drive.setWeightedDrivePower(new Pose2d(-0.2,0,0));}
            if(gamepad1.dpad_left){drive.setWeightedDrivePower(new Pose2d(0,0.3,0));}
            if(gamepad1.dpad_right){drive.setWeightedDrivePower(new Pose2d(0,-0.3,0));}
            if(gamepad1.left_bumper){drive.setWeightedDrivePower(new Pose2d(0,0,0.2));}
            if(gamepad1.right_bumper){drive.setWeightedDrivePower(new Pose2d(0,0,-0.2));}
            drive.update();


        }
    }
}
