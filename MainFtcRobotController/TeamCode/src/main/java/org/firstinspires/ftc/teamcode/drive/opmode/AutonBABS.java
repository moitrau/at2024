package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ATC;
import org.firstinspires.ftc.teamcode.drive.ATE;
import org.firstinspires.ftc.teamcode.drive.ATRoboTimer;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "02_AutonBlueAllianceBlueSample")
public class AutonBABS extends LinearOpMode {


    ATE.ClawState clawState = ATE.ClawState.CATCH;
    ATE.ClawArmState clawArmState = ATE.ClawArmState.BASE;
    ATE.ClawWristState clawWristState = ATE.ClawWristState.BASE;
    ATE.IntakeWheelsState intakeWheelsState = ATE.IntakeWheelsState.HALT;
    ATE.IntakeWristState intakeWristState = ATE.IntakeWristState.BASE;
    ATE.HorizontalSliderState hSliderState = ATE.HorizontalSliderState.BASE;
    ATE.VerticalSliderState vSliderState = ATE.VerticalSliderState.BASE;
    ATE.SampleSensorState sampleSensorState = ATE.SampleSensorState.NONE;

    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor hSlider;
    private DcMotor vSlider;
    private DcMotor rightHook;
    private DcMotor leftHook;
    private Servo clawArm;
    private Servo clawWrist;
    private Servo claw;
    private Servo intakeWrist;
    private Servo intakeLW;
    private Servo intakeRW;
    private ColorSensor sampleSensor;
    private TouchSensor vtSensor;
    private TouchSensor htSensor;

    double clawCatchTightPose = ATC.clawCatchTightPose;
    double clawCatchLoosePose = ATC.clawCatchLoosePose;
    double clawReleasePose = ATC.clawReleasePose;

    double clawWristBasePose = ATC.clawWristBasePose;
    double clawWristIntakePose = ATC.clawWristIntakePose;
    double clawWristHangPose = ATC.clawWristHangPose;
    double clawWristDropPose = ATC.clawWristDropPose;
    double clawWristWallPose = ATC.clawWristWallPose;

    double clawArmBasePose = ATC.clawArmBasePose;
    double clawArmIntakePose = ATC.clawArmIntakePose;
    double clawArmHangPose = ATC.clawArmHangPose;
    double clawArmDropPose = ATC.clawArmDropPose;
    double clawArmWallPose = ATC.clawArmWallPose;

    double intakeWristBasePose = ATC.intakeWristBasePose;
    double intakeWristConsumePose = ATC.intakeWristConsumePose;
    double intakeWristIntakePose = ATC.intakeWristIntakePose;

    double intakeWheelHaltPose = ATC.intakeWheelHaltPose;
    double intakeWheelRunPose = ATC.intakeWristRunPose;

    int vSliderBasePose = ATC.vSliderBasePose;
    int vSliderSubmHighPose = ATC.vSliderSubmHighPose;
    int vSliderSubmLowPose = ATC.vSliderSubmLowPose;
    int vSliderBaskHighPose = ATC.vSliderBaskHighPose;
    int vSliderBaskLowPose = ATC.vSliderBaskLowPose;
    int vSliderVelocity = ATC.vSliderVelocity;
    int vSliderCurrentPose = 0;

    int hSliderBasePose = ATC.hSliderBasePose;
    int hSliderMinPose = ATC.hSliderMinPose;
    int hSliderMaxPose = ATC.hSliderMaxPose;
    int hSliderVelocity = ATC.hSliderVelocity;

    private double intakeWristTimer = 0;
    private double hSliderTimer = 0;
    private double vSliderTimer = 0;

    ATRoboTimer consumeTimer = new ATRoboTimer();
    ATRoboTimer clawTimer = new ATRoboTimer();
    ATRoboTimer grabTimer = new ATRoboTimer();
    ATRoboTimer pickTimer = new ATRoboTimer();
    ATRoboTimer dropTimer = new ATRoboTimer();
    ATRoboTimer outtakeTimer = new ATRoboTimer();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//Hardware Initialization
        rightRear = hardwareMap.get(DcMotor.class, "rearRight");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "rearLeft");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        hSlider = hardwareMap.get(DcMotor.class, "horizontalSlider");
        hSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlider = hardwareMap.get(DcMotor.class, "verticalSlider");
        vSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHook = hardwareMap.get(DcMotor.class, "leftHook");
        rightHook = hardwareMap.get(DcMotor.class, "rightHook");
        clawArm = hardwareMap.get(Servo.class, "clawArm");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeLW = hardwareMap.get(Servo.class, "intakeLW");
        intakeRW = hardwareMap.get(Servo.class, "intakeRW");
        htSensor = hardwareMap.get(TouchSensor.class, "htSensor");
        vtSensor = hardwareMap.get(TouchSensor.class, "vtSensor");
        sampleSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


//Set Drive Mode
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//Initialize Poses
        claw.setPosition(clawCatchTightPose);
        clawArm.setPosition(clawArmBasePose);
        clawWrist.setPosition(clawWristBasePose);
        intakeLW.setPosition(intakeWheelHaltPose);
        intakeRW.setPosition(intakeWheelHaltPose);
        intakeWrist.setPosition(intakeWristBasePose);
        sleep(500);
        vSlider.setDirection(DcMotor.Direction.REVERSE);
        hSlider.setDirection(DcMotor.Direction.REVERSE);
        resetSlider(hSlider, htSensor, 5);
        resetSlider(vSlider, vtSensor, 5);


        waitForStart();

        if (isStopRequested()) return;


        Pose2d startPose;
        startPose = new Pose2d(-9, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        /*
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
                .build();*/

//Drop First Yellow Start
        TrajectorySequence trajSequence = drive.trajectorySequenceBuilder(startPose).setReversed(true)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-9,37.5,Math.toRadians(90)))
                .addDisplacementMarker(1, () -> {
                    setSlider(vSlider, vSliderSubmHighPose, vSliderVelocity);
                    claw.setPosition(clawCatchTightPose);
                    clawWrist.setPosition(clawWristHangPose);
                    clawArm.setPosition(clawArmHangPose);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);

        claw.setPosition(clawCatchTightPose);
        clawWrist.setPosition(clawWristHangPose);
        clawArm.setPosition(clawArmHangPose);
        setSlider(vSlider,ATC.vSliderSubmLowPose,1000);

        while (vSlider.getCurrentPosition() > ATC.vSliderSubmHighPose-300) {
            telemetry.addData("Hang First Specimen vSlider Current Position1", vSlider.getCurrentPosition());
            telemetry.update();
        }

        clawArm.setPosition(clawArmWallPose);
        clawWrist.setPosition(clawWristDropPose);

        while (vSlider.getCurrentPosition() > ATC.vSliderSubmHighPose-500 ) {
            telemetry.addData("Hang First Specimen vSlider Current Position2", vSlider.getCurrentPosition());
            telemetry.update();
        }

        claw.setPosition(clawReleasePose);
        clawArm.setPosition(clawArmBasePose);
        clawWrist.setPosition(clawWristBasePose);

        vSliderState = ATE.VerticalSliderState.EXTENDED;

        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-50, 46, Math.toRadians(-90)), Math.toRadians(-90))
                .addDisplacementMarker(40, () -> {
                    setSlider(hSlider, hSliderMinPose, hSliderVelocity);
                    clawArm.setPosition(clawArmBasePose);
                    clawWrist.setPosition(clawWristBasePose);
                    claw.setPosition(clawCatchTightPose);
                    setSlider(vSlider, 0, vSliderVelocity);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);
        while (hSlider.isBusy()) {
            if (hSliderMinPose - hSlider.getCurrentPosition() < 1) {
                break;
            }
        }
        intakeLW.setDirection(Servo.Direction.REVERSE);
        intakeRW.setDirection(Servo.Direction.FORWARD);
        intakeLW.setPosition(0.9);
        intakeRW.setPosition(0.9);
        intakeWrist.setPosition(ATC.intakeWristIntakePose);
        sleep(500);
        setSlider(hSlider, hSliderMaxPose, hSliderVelocity);
        while (hSlider.isBusy()) {
            sampleSensorState = getSampleSensorState();
            if (hSliderMaxPose - hSlider.getCurrentPosition() < 5 || sampleSensorState == ATE.SampleSensorState.YELLOW || sampleSensorState == ATE.SampleSensorState.BLUE) {
                break;
            }
        }
        //setSlider(hSlider, 0, vSliderVelocity);
        intakeWrist.setPosition(ATC.intakeWristBasePose);
        /*while (hSlider.isBusy()) {
            if (hSlider.getCurrentPosition() < 5) {
                break;
            }
        }*/
        resetSlider(vSlider, vtSensor, 2);
        resetSlider(hSlider, htSensor, 2);
        intakeWrist.setPosition(ATC.intakeWristConsumePose);
        intakeLW.setPosition(0.5);
        intakeRW.setPosition(0.5);
        sleep(500);
        intakeWrist.setPosition(ATC.intakeWristPickIntakePose);
        clawWrist.setPosition(clawWristBasePose);
        clawArm.setPosition(clawArmIntakePose);
        claw.setPosition(clawReleasePose);
        sleep(750);
        claw.setPosition(clawCatchLoosePose);
        sleep(500);
        clawArm.setPosition(clawArmWallPose);
        sleep(250);
        clawWrist.setPosition(clawWristWallPose);
        sleep(250);
        claw.setPosition(clawReleasePose);
////Consume first blue ends
////Consume second blue starts
        vSliderState = ATE.VerticalSliderState.EXTENDED;

        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-60, 46, Math.toRadians(-90)))
                .addDisplacementMarker(1, () -> {
                    setSlider(hSlider, hSliderMinPose, hSliderVelocity);
                    clawArm.setPosition(clawArmBasePose);
                    clawWrist.setPosition(clawWristBasePose);
                    claw.setPosition(clawCatchTightPose);
                    setSlider(vSlider, 0, vSliderVelocity);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);
        while (hSlider.isBusy()) {
            if (hSliderMinPose - hSlider.getCurrentPosition() < 1) {
                break;
            }
        }
        intakeLW.setDirection(Servo.Direction.REVERSE);
        intakeRW.setDirection(Servo.Direction.FORWARD);
        intakeLW.setPosition(0.9);
        intakeRW.setPosition(0.9);
        intakeWrist.setPosition(ATC.intakeWristIntakePose);
        sleep(500);
        setSlider(hSlider, hSliderMaxPose, hSliderVelocity);
        while (hSlider.isBusy()) {
            sampleSensorState = getSampleSensorState();
            if (hSliderMaxPose - hSlider.getCurrentPosition() < 5 || sampleSensorState == ATE.SampleSensorState.YELLOW || sampleSensorState == ATE.SampleSensorState.BLUE) {
                break;
            }
        }
        //setSlider(hSlider, 0, vSliderVelocity);
        intakeWrist.setPosition(ATC.intakeWristBasePose);
        /*while (hSlider.isBusy()) {
            if (hSlider.getCurrentPosition() < 5) {
                break;
            }
        }*/
        resetSlider(vSlider, vtSensor, 2);
        resetSlider(hSlider, htSensor, 2);
        intakeWrist.setPosition(ATC.intakeWristConsumePose);
        intakeLW.setPosition(0.5);
        intakeRW.setPosition(0.5);
        sleep(500);
        intakeWrist.setPosition(ATC.intakeWristPickIntakePose);
        clawWrist.setPosition(clawWristBasePose);
        clawArm.setPosition(clawArmIntakePose);
        claw.setPosition(clawReleasePose);
        sleep(750);
        claw.setPosition(clawCatchLoosePose);
        sleep(500);
        clawArm.setPosition(clawArmWallPose);
        sleep(250);
        clawWrist.setPosition(clawWristWallPose);
        sleep(250);
        claw.setPosition(clawReleasePose);

///Consume second blue ends
//Pickup second blue begins

        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-60, 56, Math.toRadians(-90)))
                .addDisplacementMarker(1, () -> {
                    claw.setPosition(clawCatchTightPose);
                    setSlider(vSlider, ATC.vSliderWallLiftPose, vSliderVelocity);
                })
                .build();


        sleep(30000);
    }

    private void setSlider(DcMotor slider, int targetPose, int velocity) {
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setTargetPosition(targetPose);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) slider).setVelocity(velocity);
    }

    private void resetSlider(DcMotor slider, TouchSensor tSensor, double maxResetTime) {

        ATRoboTimer sliderTimer = new ATRoboTimer();
        sliderTimer.startTimer();

        while ((!tSensor.isPressed()) && sliderTimer.elapsedTime() <= maxResetTime) {
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("MaxTimer", maxResetTime);
            telemetry.addData("VTimer", sliderTimer.elapsedTime());
            telemetry.update();
            slider.setPower(-1);
        }
        slider.setPower(0);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private ATE.SampleSensorState getSampleSensorState() {
        ((NormalizedColorSensor) sampleSensor).setGain(2);
        double distance = ((DistanceSensor) sampleSensor).getDistance(DistanceUnit.CM);

        double hue = Double.parseDouble(JavaUtil.formatNumber(JavaUtil.colorToHue(((NormalizedColorSensor) sampleSensor).getNormalizedColors().toColor()), 0));

        if (distance <= 4 && hue >= 200 && hue <= 240) {
            return ATE.SampleSensorState.BLUE;
        } else if (distance <= 4 && hue >= 80 && hue <= 100) {
            return ATE.SampleSensorState.YELLOW;
        } else if (distance <= 4 && hue >= 0 && hue <= 60) {
            return ATE.SampleSensorState.RED;
        } else {
            return ATE.SampleSensorState.NONE;
        }
    }
}