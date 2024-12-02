package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import org.firstinspires.ftc.teamcode.ATCommons;
import org.firstinspires.ftc.teamcode.CalibrationFromLimeLight;
import org.firstinspires.ftc.teamcode.drive.ATC;
import org.firstinspires.ftc.teamcode.drive.ATCache;
import org.firstinspires.ftc.teamcode.drive.ATE;
import org.firstinspires.ftc.teamcode.drive.ATRoboTimer;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveFaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "04_BlueSubmers4BS")
public class AutonBASubmBS4 extends LinearOpMode {


    ATE.ClawState clawState = ATE.ClawState.CATCH;
    ATE.ClawArmState clawArmState = ATE.ClawArmState.BASE;
    ATE.ClawWristState clawWristState = ATE.ClawWristState.BASE;
    ATE.IntakeWheelsState intakeWheelsState = ATE.IntakeWheelsState.HALT;
    ATE.IntakeWristState intakeWristState = ATE.IntakeWristState.BASE;
    ATE.HorizontalSliderState hSliderState = ATE.HorizontalSliderState.BASE;
    ATE.VerticalSliderState vSliderState = ATE.VerticalSliderState.BASE;
    ATE.SampleSensorState presampleSensorState = ATE.SampleSensorState.NONE;

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
    private ColorSensor presampleSensor;
    private DistanceSensor distanceSensor;
    private TouchSensor vtSensor;
    private TouchSensor htSensor;
    private Limelight3A limelight;
    private CalibrationFromLimeLight calibrationFromLimeLight;
    private ATCommons atCommons;

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
    double intakeWheelRunPose = ATC.intakeWheelRunPose;

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

    double rrllDeltaY = 0.0;
    double rrllDeltaHeading = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveFaster drive = new SampleMecanumDriveFaster(hardwareMap);

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
        presampleSensor = hardwareMap.get(ColorSensor.class, "precolorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


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
        atCommons = new ATCommons();
        atCommons.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose;
        startPose = new Pose2d(-9, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

//Hang first blue Start
        TrajectorySequence trajSequence = drive.trajectorySequenceBuilder(startPose).setReversed(true)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-9,39,Math.toRadians(90)))
                .addDisplacementMarker(1, () -> {
                    setSlider(vSlider, vSliderSubmHighPose, vSliderVelocity);
                    claw.setPosition(clawCatchTightPose);
                    clawWrist.setPosition(clawWristHangPose);
                    clawArm.setPosition(clawArmHangPose);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);

        calibrationFromLimeLight = new CalibrationFromLimeLight();
        calibrationFromLimeLight.LimeLight3A(0, 0, 0, 0.5, hardwareMap);

        double[] resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIghtRaw();
        if(resultCordinates[1] > 10) {
            //limelightCorrectionActive = true;
            rrllDeltaY = trajSequence.end().getY() - resultCordinates[1];
            rrllDeltaHeading = Math.toDegrees(trajSequence.end().getHeading()) - resultCordinates[2];
        }

        telemetry.addData("rrllDeltaY",rrllDeltaY);
        telemetry.addData("rrllDeltaHeading",rrllDeltaHeading);
        telemetry.update();

        atCommons.selfAdjust(ATC.submAdjustDistance,ATC.submAdjustMaxTime,ATC.submAdjustPower);

        claw.setPosition(clawCatchTightPose);
        clawWrist.setPosition(clawWristHangPose);
        clawArm.setPosition(clawArmHangPose);
        setSlider(vSlider,ATC.vSliderSubmPullUpPose,ATC.vSliderHangVelocity);

        while (vSlider.getCurrentPosition() <=  ATC.vSliderSubmPullUpPose-120) {
            atCommons.startSelfAdjustAsync(ATC.submAdjustDistance,ATC.submAdjustPower);
            telemetry.addData("Hang First Specimen vSlider Current Position1", vSlider.getCurrentPosition());
            //telemetry.update();
        }

        claw.setPosition(clawReleasePose);
        clawArm.setPosition(clawArmBasePose);
        clawWrist.setPosition(clawWristBasePose);
        resetSlider(vSlider, vtSensor, 2);
//Hang first blue ends
//sleep(100000);
//Push two Blue
        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())

                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-42,42,Math.toRadians(-110)),Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60.0))
                .addDisplacementMarker(3, () -> {
                    setSlider(hSlider, hSliderMinPose, hSliderVelocity);

                })
                .addDisplacementMarker(40, () -> {
                    intakeLW.setDirection(Servo.Direction.REVERSE);
                    intakeRW.setDirection(Servo.Direction.FORWARD);
                    intakeLW.setPosition(0.9);
                    intakeRW.setPosition(0.9);
                    intakeWrist.setPosition(ATC.intakeWristIntakePose);
                    //sleep(500);
                    setSlider(hSlider, hSliderMaxPose, 2000);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);
        while (hSlider.isBusy()) {
            presampleSensorState = getPreSampleSensorState(presampleSensor);
            if (hSliderMaxPose - hSlider.getCurrentPosition() < 5 || presampleSensorState == ATE.SampleSensorState.BLUE) {
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                intakeWrist.setPosition(ATC.intakeWristBasePose);
                break;
            }
        }

        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .turn(Math.toRadians(-120))
                .addDisplacementMarker(20, () -> {
                    setSlider(hSlider, hSliderMaxPose-50, 10000);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);

        intakeWrist.setPosition(ATC.intakeWristOuttakePose);
        intakeLW.setDirection(Servo.Direction.FORWARD);
        intakeRW.setDirection(Servo.Direction.REVERSE);
        intakeLW.setPosition(0.9);
        intakeRW.setPosition(0.9);
        sleep(300);
        intakeLW.setPosition(0.5);
        intakeRW.setPosition(0.5);
        setSlider(hSlider, hSliderMaxPose-900, 10000);
        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .turn(Math.toRadians(90))
                .addDisplacementMarker(10, () -> {
                    intakeLW.setDirection(Servo.Direction.REVERSE);
                    intakeRW.setDirection(Servo.Direction.FORWARD);
                    intakeLW.setPosition(0.9);
                    intakeRW.setPosition(0.9);
                    intakeWrist.setPosition(ATC.intakeWristIntakePose);
                    //sleep(500);
                    setSlider(hSlider, hSliderMaxPose, 2000);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);

        while (hSlider.isBusy()) {
            presampleSensorState = getPreSampleSensorState(presampleSensor);
            if (hSliderMaxPose - hSlider.getCurrentPosition() < 5 || presampleSensorState == ATE.SampleSensorState.BLUE) {
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                break;
            }
        }
        //setSlider(hSlider, hSliderMaxPose-300, 10000);
        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .turn(Math.toRadians(-100))
                .addDisplacementMarker(20, () -> {
                    setSlider(hSlider, hSliderMaxPose-50, 10000);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);
        intakeWrist.setPosition(ATC.intakeWristOuttakePose);
        intakeLW.setDirection(Servo.Direction.FORWARD);
        intakeRW.setDirection(Servo.Direction.REVERSE);
        intakeLW.setPosition(0.9);
        intakeRW.setPosition(0.9);
        sleep(300);
        intakeLW.setPosition(0.5);
        intakeRW.setPosition(0.5);
        resetSlider(hSlider, htSensor, 2);
        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .turn(Math.toRadians(140))
                .lineToLinearHeading(new Pose2d(-45,59,Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60.0))
                .addDisplacementMarker(1, () -> {
                    clawArm.setPosition(clawArmWallPose);
                    clawWrist.setPosition(clawWristWallPose);
                    claw.setPosition(clawReleasePose);
                })
                .build();

        drive.followTrajectorySequence(trajSequence);

        atCommons.selfAdjust(ATC.wallAdjustDistance,ATC.wallAdjustMaxTime,0.35);
//Pick from wall and hang 2nd Blue

        claw.setPosition(clawCatchTightPose);
        sleep(200);
        clawWrist.setPosition(clawWristDropPose);
        setSlider(vSlider,ATC.vSliderWallLiftPose,vSliderVelocity);

        stop();
        sleep(100000);
        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-0,44,Math.toRadians(80)),Math.toRadians(-60),
                        SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60.0))
                .build();
        drive.followTrajectorySequence(trajSequence);
        clawArm.setPosition(clawArmHangPose);
        clawWrist.setPosition(clawWristHangPose);
        clawArm.setPosition(clawArmHangPose);
        calibrationFromLimeLight = new CalibrationFromLimeLight();
        calibrationFromLimeLight.LimeLight3A(0, 0, 0, 0.5, hardwareMap);
        int i = 0;
        resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIght();
        while(i<100){
            i++;
            resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIght();
            if(resultCordinates[1] > 10.0){
                break;
            }
        }

        telemetry.addData("resultCordinates[1]",resultCordinates[1]);
        telemetry.addData("resultCordinates[2]",resultCordinates[2]);

        Pose2d correctedPose = new Pose2d(trajSequence.end().getX(),resultCordinates[1]+rrllDeltaY,Math.toRadians(resultCordinates[2]+rrllDeltaHeading));
        drive.setPoseEstimate(correctedPose);

        telemetry.addData("X",drive.getPoseEstimate().getX());
        telemetry.addData("Y",drive.getPoseEstimate().getY());
        telemetry.addData("Heading",Math.toDegrees(drive.getPoseEstimate().getHeading()));


        //sleep(500000);
        setSlider(vSlider, vSliderSubmHighPose, vSliderVelocity);
        trajSequence = drive.trajectorySequenceBuilder(correctedPose)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-7,39,Math.toRadians(90)))
                .addDisplacementMarker(1, () -> {
                    //setSlider(vSlider, vSliderSubmHighPose, vSliderVelocity);
                    //claw.setPosition(clawCatchTightPose);
                    //clawWrist.setPosition(clawWristHangPose);
                    //clawArm.setPosition(clawArmHangPose);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);
        atCommons.selfAdjust(ATC.submAdjustDistance,ATC.submAdjustMaxTime,ATC.submAdjustPower);


        /*calibrationFromLimeLight = new CalibrationFromLimeLight();
        calibrationFromLimeLight.LimeLight3A(0, 0, 0, 0.5, hardwareMap);
        int i = 0;
        double[] resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIght();
        while(i<100){
            i++;
            resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIght();
            if(resultCordinates[1] > 10.0){
                break;
            }
        }
        telemetry.addData("X Coordinate", resultCordinates[0]);
        telemetry.addData("Y Coordinate", resultCordinates[1]);
        telemetry.addData("Heading (Yaw)", resultCordinates[2]);
        telemetry.update();

        sleep(5000);
        if(resultCordinates[1] > 10){
            resultCordinates[1]=resultCordinates[1]-5.7;
            resultCordinates[2]=resultCordinates[2];
        }else{
            resultCordinates[1]=trajSequence.end().getY();
            resultCordinates[2]=trajSequence.end().getHeading();
        }

        startPose = new Pose2d(trajSequence.end().getX(), resultCordinates[1], Math.toRadians(resultCordinates[2]));
        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-7,39,Math.toRadians(90)))
                .addDisplacementMarker(30, () -> {
                    setSlider(vSlider, vSliderSubmHighPose+50, vSliderVelocity);
                    claw.setPosition(clawCatchTightPose);
                    clawWrist.setPosition(clawWristHangPose);
                    clawArm.setPosition(clawArmHangPose);
                })
                .build();
        drive.followTrajectorySequence(trajSequence);*/

        setSlider(vSlider,ATC.vSliderSubmPullUpPose,ATC.vSliderHangVelocity);

        while (vSlider.getCurrentPosition() <=  ATC.vSliderSubmPullUpPose-50) {
            atCommons.startSelfAdjustAsync(ATC.submAdjustDistance,ATC.submAdjustPower);
            telemetry.addData("Hang First Specimen vSlider Current Position1", vSlider.getCurrentPosition());
            //telemetry.update();
        }
        claw.setPosition(clawReleasePose);
        clawArm.setPosition(clawArmBasePose);
        clawWrist.setPosition(clawWristBasePose);
        resetSlider(vSlider, vtSensor, 2);

//sleep(20000);
//Go Back to wall to pick 3rd blue


        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())

                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-47,50,Math.toRadians(-90)),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .addDisplacementMarker(20, () -> {
                    clawArm.setPosition(clawArmWallPose);
                    clawWrist.setPosition(clawWristWallPose);
                    claw.setPosition(clawReleasePose);
                })
                .lineToLinearHeading(new Pose2d(-47,60,Math.toRadians(-90)))
                .build();
        drive.followTrajectorySequence(trajSequence);
        atCommons.selfAdjust(ATC.wallAdjustDistance,ATC.wallAdjustMaxTime,ATC.wallAdjustPower);
//Pick from wall and hang 3nd Blue

        claw.setPosition(clawCatchTightPose);
        sleep(200);
        clawWrist.setPosition(clawWristDropPose);
        setSlider(vSlider,ATC.vSliderWallLiftPose,vSliderVelocity);

        trajSequence = drive.trajectorySequenceBuilder(trajSequence.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(0,44,Math.toRadians(80)),Math.toRadians(-60),
                        SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        drive.followTrajectorySequence(trajSequence);
        clawArm.setPosition(clawArmHangPose);
        clawWrist.setPosition(clawWristHangPose);
        clawArm.setPosition(clawArmHangPose);
        calibrationFromLimeLight = new CalibrationFromLimeLight();
        calibrationFromLimeLight.LimeLight3A(0, 0, 0, 0.5, hardwareMap);
        i = 0;
        resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIght();
        while(i<100){
            i++;
            resultCordinates = calibrationFromLimeLight.CalibratePoseWithLimelIght();
            if(resultCordinates[1] > 10.0){
                break;
            }
        }
        telemetry.addData("resultCordinates[1]",resultCordinates[1]);
        telemetry.addData("resultCordinates[2]",resultCordinates[2]);

        correctedPose = new Pose2d(trajSequence.end().getX(),resultCordinates[1]+rrllDeltaY,Math.toRadians(resultCordinates[2]+rrllDeltaHeading));
        drive.setPoseEstimate(correctedPose);

        telemetry.addData("X",drive.getPoseEstimate().getX());
        telemetry.addData("Y",drive.getPoseEstimate().getY());
        telemetry.addData("Heading",Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.update();


        setSlider(vSlider, vSliderSubmHighPose, vSliderVelocity);
        trajSequence = drive.trajectorySequenceBuilder(correctedPose)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-5.0,39,Math.toRadians(90)))
                .addDisplacementMarker(1, () -> {
                    //setSlider(vSlider, vSliderSubmHighPose, vSliderVelocity);

                })
                .build();
        drive.followTrajectorySequence(trajSequence);
        atCommons.selfAdjust(ATC.submAdjustDistance,ATC.submAdjustMaxTime,ATC.submAdjustPower);

        setSlider(vSlider,ATC.vSliderSubmPullUpPose,ATC.vSliderHangVelocity);

        while (vSlider.getCurrentPosition() <=  ATC.vSliderSubmPullUpPose-50) {
            atCommons.startSelfAdjustAsync(ATC.submAdjustDistance,ATC.submAdjustPower);
            telemetry.addData("Hang First Specimen vSlider Current Position1", vSlider.getCurrentPosition());
            //telemetry.update();
        }
        claw.setPosition(clawReleasePose);
        clawArm.setPosition(clawArmBasePose);
        clawWrist.setPosition(clawWristBasePose);
        resetSlider(vSlider, vtSensor, 2);


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
            //telemetry.addData("MaxTimer", maxResetTime);
            //telemetry.addData("VTimer", sliderTimer.elapsedTime());
            //telemetry.update();
            slider.setPower(-1);
        }
        slider.setPower(0);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private ATE.SampleSensorState getPreSampleSensorState(ColorSensor colSensor){
        ((NormalizedColorSensor) colSensor).setGain(2);
        double distance = ((DistanceSensor) colSensor).getDistance(DistanceUnit.CM);

        double hue = Double.parseDouble(JavaUtil.formatNumber(JavaUtil.colorToHue(((NormalizedColorSensor) colSensor).getNormalizedColors().toColor()), 0));

        if (distance <= 2.8 && hue >= 200 && hue <= 250) {
            return ATE.SampleSensorState.BLUE;
        } else if (distance <= 2.8 && hue >= 55 && hue <= 100) {
            return ATE.SampleSensorState.YELLOW;
        } else if (distance <= 2.8 && hue >= 0 && hue <= 50) {
            return ATE.SampleSensorState.RED;
        } else {
            return ATE.SampleSensorState.NONE;
        }
    }
}