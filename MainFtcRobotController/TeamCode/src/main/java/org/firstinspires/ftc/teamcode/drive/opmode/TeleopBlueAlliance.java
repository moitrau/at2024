package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@TeleOp(name="01_TeleopBlueAlliance")
public class TeleopBlueAlliance extends LinearOpMode {


    ATE.ClawState clawState = ATE.ClawState.CATCH;
    ATE.ClawArmState clawArmState = ATE.ClawArmState.BASE;
    ATE.ClawWristState clawWristState = ATE.ClawWristState.BASE;
    ATE.IntakeWheelsState intakeWheelsState = ATE.IntakeWheelsState.HALT;
    ATE.IntakeWristState intakeWristState = ATE.IntakeWristState.BASE;
    ATE.HorizontalSliderState hSliderState = ATE.HorizontalSliderState.BASE;
    ATE.VerticalSliderState vSliderState = ATE.VerticalSliderState.BASE;
    ATE.SampleSensorState sampleSensorState = ATE.SampleSensorState.NONE;
    ATE.HangerState hangerState = ATE.HangerState.IDLE;

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
    int vSliderCurrentPose=0;

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        vSliderTimer = resetTimer();
        vSliderTimer = startTimer();

        vSlider.setDirection(DcMotor.Direction.REVERSE);
        while((!vtSensor.isPressed()) && elapsedTime(vSliderTimer) <= ATC.vSliderMaxTime){
            vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("MaxTimer", ATC.vSliderMaxTime);
            telemetry.addData("VTimer", elapsedTime(vSliderTimer));
            telemetry.update();
            vSlider.setPower(-1);
        }
        vSlider.setPower(0);
        vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hSliderTimer = resetTimer();
        hSliderTimer = startTimer();
        while((!htSensor.isPressed()) && elapsedTime(hSliderTimer) <= ATC.hSliderMaxTime){
            hSliderState = ATE.HorizontalSliderState.BASE;
            intakeWristState = ATE.IntakeWristState.CONSUME;
            hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hSlider.setPower(1);
        }
        hSlider.setPower(0);
        hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlider.setDirection(DcMotor.Direction.REVERSE);
        hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//Wait till Play button is clicked on DriverHub
        waitForStart();

        while (!isStopRequested()) {

            if(htSensor.isPressed()){
                //If  Horizontal Slider touches touch sensor, wrist should go to consume position
                if(intakeWristState == ATE.IntakeWristState.BASE) {
                    intakeWristState = ATE.IntakeWristState.CONSUME;
                    intakeWrist.setPosition(ATC.intakeWristConsumePose);
                    hSlider.setPower(0);
                }
                /* Below lines are to automate activation of grab,catch,pick when the horizontal slider
                comes back to base touching the sensor after extension when it has yellow or blue sample */
                if(hSliderState == ATE.HorizontalSliderState.EXTENDED && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                    clawArmState = ATE.ClawArmState.PICK_INTAKE;
                    clawWristState = ATE.ClawWristState.PICK_INTAKE;
                    intakeWristState = ATE.IntakeWristState.PICK_INTAKE;
                    consumeTimer.startTimer();
                }
                //Change slider state to BASE and reset encoder
                hSliderState = ATE.HorizontalSliderState.BASE;
                hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            //This is to make horizontal slider go back inside
            if (gamepad1.dpad_left && hSliderState != ATE.HorizontalSliderState.BASE) {
                intakeWrist.setPosition(ATC.intakeWristBasePose);
                hSlider.setPower(-0.7);
             //Below if condition is to extend horizontal slider outside
            }else if(gamepad1.dpad_right && hSlider.getCurrentPosition()<=ATC.hSliderMaxPose){
                hSliderState = ATE.HorizontalSliderState.EXTENDED;
                intakeWristState = ATE.IntakeWristState.BASE;
                intakeWrist.setPosition(ATC.intakeWristBasePose);
                hSlider.setPower(0.7);
            //If no buttons are pressed move intake wrist to base position
            }else if(hangerState == ATE.HangerState.IDLE){
                intakeWrist.setPosition(ATC.intakeWristBasePose);
                hSlider.setPower(0);
            }
            //If horizontal slider state is BASE and Intake Wrist state is CONSUME, set horizontal slider power to 0 always
            if( hSliderState == ATE.HorizontalSliderState.BASE && intakeWristState == ATE.IntakeWristState.CONSUME){
                intakeWrist.setPosition(ATC.intakeWristConsumePose);
                hSlider.setPower(0);
            }
            //If Vertical Slider touch sensor is pressed, and the state is EXTENDED or SUBM_HIGH or SUBM_LOW, set  STATE to BASE
            // also set power to 0 and reset encoder
            if(vtSensor.isPressed()){
                if(vSliderState == ATE.VerticalSliderState.EXTENDED || vSliderState == ATE.VerticalSliderState.SUBM_HIGH || vSliderState == ATE.VerticalSliderState.SUBM_LOW ) {
                    vSliderState = ATE.VerticalSliderState.BASE;
                    vSlider.setPower(0);
                    vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }


//Intake-Outtake code for sample starts
            //Below code is to start outtake
            if ( ((gamepad1.y && hSlider.getCurrentPosition() >= ATC.hSliderMinPose) || sampleSensorState == ATE.SampleSensorState.RED) && hSliderState == ATE.HorizontalSliderState.EXTENDED ) {
                intakeLW.setDirection(Servo.Direction.FORWARD);
                intakeRW.setDirection(Servo.Direction.REVERSE);
                intakeLW.setPosition(0.9);
                intakeRW.setPosition(0.9);
                sampleSensorState = getSampleSensorState();
                intakeWrist.setPosition(ATC.intakeWristOuttakePose);
            //Below if condition is to start intake
            }else if ((gamepad1.x && hSlider.getCurrentPosition() >= ATC.hSliderMinPose) && sampleSensorState == ATE.SampleSensorState.NONE && hSliderState == ATE.HorizontalSliderState.EXTENDED) {
                intakeLW.setDirection(Servo.Direction.REVERSE);
                intakeRW.setDirection(Servo.Direction.FORWARD);
                intakeLW.setPosition(0.9);
                intakeRW.setPosition(0.9);
                intakeWrist.setPosition(ATC.intakeWristIntakePose);
            }else if ((sampleSensorState == ATE.SampleSensorState.YELLOW || sampleSensorState == ATE.SampleSensorState.BLUE ) && hSliderState == ATE.HorizontalSliderState.EXTENDED){
                hSlider.setPower(-1);
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                intakeWrist.setPosition(ATC.intakeWristBasePose);
            }else if( sampleSensorState == ATE.SampleSensorState.NONE && hSliderState == ATE.HorizontalSliderState.EXTENDED){
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                intakeWrist.setPosition(ATC.intakeWristBasePose);
            }

//Sample sensor code starts
            if(sampleSensorState == ATE.SampleSensorState.NONE) {
                sampleSensorState = getSampleSensorState();
            }
//Sample sensor code ends

            if(clawArmState == ATE.ClawArmState.PICK_INTAKE && clawWristState == ATE.ClawWristState.PICK_INTAKE && intakeWristState == ATE.IntakeWristState.PICK_INTAKE ) {
                if(consumeTimer.isActive) {
                    intakeWrist.setPosition(ATC.intakeWristConsumePose);
                    if (consumeTimer.elapsedTime() >= ATC.consumeMaxTime){
                        consumeTimer.stopTimer();
                        grabTimer.startTimer();
                    }
                }
                if(grabTimer.isActive) {
                    intakeWrist.setPosition(ATC.intakeWristPickIntakePose);
                    clawWrist.setPosition(clawWristBasePose);
                    clawArm.setPosition(clawArmIntakePose);
                    claw.setPosition(clawReleasePose);

                    if (grabTimer.elapsedTime() >= ATC.grabMaxTime){
                        grabTimer.stopTimer();
                        clawTimer.startTimer();
                    }
                }
                if (clawTimer.isActive) {
                    clawWrist.setPosition(clawWristIntakePose);
                    claw.setPosition(clawCatchLoosePose);
                    if (clawTimer.elapsedTime() >= ATC.clawMaxTime){
                        clawTimer.stopTimer();
                        pickTimer.startTimer();
                    }
                }
                if(pickTimer.isActive){
                    clawArm.setPosition(clawArmBasePose);
                    if (pickTimer.elapsedTime() >= ATC.pickMaxTime){
                        clawWrist.setPosition(clawWristIntakePose);
                        pickTimer.stopTimer();
                        clawArmState = ATE.ClawArmState.BASE;
                        clawWristState = ATE.ClawWristState.BASE;
                    }
                }
            }

//Intake-Outtake code for sample ends



//Basket Drop Code Starts
            if( gamepad2.dpad_up && gamepad2.x && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                setSlider(vSlider,vSliderBaskHighPose,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.EXTENDED;
            }
            if( gamepad2.dpad_down && gamepad2.x && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                setSlider(vSlider,vSliderBaskLowPose,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.EXTENDED;
            }
            if( gamepad2.y && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                clawWristState = ATE.ClawWristState.DROP;
                clawArmState = ATE.ClawArmState.DROP;
                sampleSensorState = ATE.SampleSensorState.NONE;
                dropTimer.startTimer();
            }
            if (clawWristState == ATE.ClawWristState.DROP && clawArmState == ATE.ClawArmState.DROP && sampleSensorState == ATE.SampleSensorState.NONE){
                if(dropTimer.isActive){
                    clawArm.setPosition(clawArmDropPose);
                    clawWrist.setPosition(clawWristDropPose);
                    if (dropTimer.elapsedTime() >= ATC.dropMaxTime){
                        dropTimer.stopTimer();
                        clawTimer.startTimer();
                    }
                }
                if(clawTimer.isActive){
                    claw.setPosition(clawReleasePose);
                    if (clawTimer.elapsedTime() >= ATC.clawMaxTime){
                        clawTimer.stopTimer();
                        clawArm.setPosition(clawArmBasePose);
                        clawWrist.setPosition(clawWristBasePose);
                        claw.setPosition(clawCatchTightPose);
                        sleep(500);
                        clawWristState = ATE.ClawWristState.BASE;
                        clawArmState = ATE.ClawArmState.BASE;
                        vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        telemetry.update();
                        vSlider.setPower(-1);
                    }
                }

            }

//Basket drop code ends

//Specimen code starts

            if(gamepad1.a && sampleSensorState == ATE.SampleSensorState.NONE && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE ){
                clawArm.setPosition(clawArmWallPose);
                clawWrist.setPosition(clawWristWallPose);
                sleep(250);
                claw.setPosition(clawReleasePose);
                clawWristState = ATE.ClawWristState.PICK_WALL;
            }
            if(gamepad1.b && sampleSensorState == ATE.SampleSensorState.NONE && clawWristState == ATE.ClawWristState.PICK_WALL && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE ){
                claw.setPosition(clawCatchTightPose);
                sleep(200);
                clawWrist.setPosition(clawWristDropPose);
                //clawArm.setPosition(clawArmDropPose);
                setSlider(vSlider,ATC.vSliderWallLiftPose,vSliderVelocity);
                clawWristState = ATE.ClawWristState.BASE;
            }


            if(gamepad2.a && sampleSensorState == ATE.SampleSensorState.NONE && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE ){
                claw.setPosition(clawCatchTightPose);
                clawWrist.setPosition(clawWristHangPose);
                clawArm.setPosition(clawArmHangPose);
                setSlider(vSlider,vSliderSubmHighPose,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.SUBM_HIGH;
                clawWristState = ATE.ClawWristState.HANG;
            }

            if(gamepad2.b && sampleSensorState == ATE.SampleSensorState.NONE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.SUBM_HIGH ){
                claw.setPosition(clawCatchTightPose);
                clawWrist.setPosition(clawWristHangPose);
                clawArm.setPosition(clawArmHangPose);
                setSlider(vSlider,ATC.vSliderSubmMidPose,1000);
                vSliderState = ATE.VerticalSliderState.SUBM_LOW;
                //clawWrist.getController().pwmDisable();
                clawArmState = ATE.ClawArmState.HANG;
            }
            if( vSlider.getCurrentPosition() <= ATC.vSliderSubmHighPose-200  && sampleSensorState == ATE.SampleSensorState.NONE  && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.SUBM_LOW ){
                clawArm.setPosition(clawArmWallPose);
                clawWrist.setPosition(clawWristDropPose);

            }
            if( vSlider.getCurrentPosition() <= ATC.vSliderSubmMidPose+10 && sampleSensorState == ATE.SampleSensorState.NONE  && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.SUBM_LOW ){
                /*TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(2)
                        .build();
                drive.followTrajectorySequence(trajSeq);
                */

                //sleep(5000);
                //claw.setPosition(clawCatchTightPose+0.05);
                //clawArm.setPosition(clawArmWallPose);
               // clawWrist.setPosition(clawWristWallPose);

                //sleep(5);
                claw.setPosition(clawReleasePose);
                //sleep(1000);
                //clawArm.setPosition(clawArmBasePose);
                //clawWrist.setPosition(clawWristBasePose);
                //clawWristState = ATE.ClawWristState.BASE;
                //clawArmState = ATE.ClawArmState.BASE;
                vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //telemetry.update();
                vSliderState = ATE.VerticalSliderState.EXTENDED;
                vSlider.setPower(-1);
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                clawWristState = ATE.ClawWristState.BASE;
                clawArmState = ATE.ClawArmState.BASE;
                clawWrist.getController().pwmEnable();
            }

/*
            if(gamepad2.right_bumper && gamepad2.y && clawWristState == ATE.ClawWristState.PICK_WALL && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE && sampleSensorState == ATE.SampleSensorState.NONE ){
                claw.setPosition(clawCatchTightPose);
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                clawWristState = ATE.ClawWristState.BASE;
            }

            if(gamepad2.right_bumper && gamepad2.b && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.EXTENDED && sampleSensorState == ATE.SampleSensorState.NONE ){
                clawArm.setPosition(clawArmHangPose);
                clawWrist.setPosition(clawWristHangPose);
                clawWristState = ATE.ClawWristState.HANG;
            }

            if(gamepad2.right_bumper && gamepad2.a && clawWristState == ATE.ClawWristState.HANG && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.EXTENDED && sampleSensorState == ATE.SampleSensorState.NONE ){
                claw.setPosition(clawReleasePose);
                clawWrist.setPosition(clawWristBasePose);
                clawArm.setPosition(clawArmBasePose);
                claw.setPosition(clawCatchTightPose);
                clawWristState = ATE.ClawWristState.BASE;
            }
//Specimen code ends
*/
//Code to hang onto submersible starts
            if (gamepad2.right_bumper && hangerState == ATE.HangerState.IDLE) {
                hangerState = ATE.HangerState.ACTIVE;
                hSlider.setPower(0);
                vSlider.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                leftHook.setPower(1);
                rightHook.setPower(1);
                intakeLW.getController().pwmDisable();
                intakeRW.getController().pwmDisable();
                intakeWrist.getController().pwmDisable();
                claw.getController().pwmDisable();
                clawWrist.getController().pwmDisable();
                clawArm.getController().pwmDisable();
            }

            if (gamepad2.left_bumper && hangerState == ATE.HangerState.ACTIVE) {
                hangerState = ATE.HangerState.IDLE;
                leftHook.setPower(0);
                rightHook.setPower(0);
                intakeLW.getController().pwmEnable();
                intakeRW.getController().pwmEnable();
                intakeWrist.getController().pwmEnable();
                claw.getController().pwmEnable();
                clawWrist.getController().pwmEnable();
                clawArm.getController().pwmEnable();

            }
//Code to hang onto submersible ends


            telemetry.addData("claw state", clawState);
            telemetry.addData("clawWrist state", clawWristState);
            telemetry.addData("clawArm state", clawArmState);
            telemetry.addData("intakeWrist state", intakeWristState);
            telemetry.addData("intakeWheels state", intakeWheelsState);
            telemetry.addData("sample sensor state", sampleSensorState);
            telemetry.addData("hSlider state", hSliderState);
            telemetry.addData("vSlider state", vSliderState);
            telemetry.addData("hslider pos", hSlider.getCurrentPosition());
            telemetry.addData("vslider pos", vSlider.getCurrentPosition());
            telemetry.addData("claw pos", claw.getPosition());
            telemetry.addData("clawArm pos", clawArm.getPosition());
            telemetry.addData("clawWrist pos", clawWrist.getPosition());
            telemetry.addData("intakeWrist pos", intakeWrist.getPosition());
            telemetry.addData("consumeTimer:",consumeTimer.elapsedTime());
            telemetry.addData("grabTimer:",grabTimer.elapsedTime());
            telemetry.addData("clawTimer:",clawTimer.elapsedTime());
            telemetry.addData("pickTimer:",pickTimer.elapsedTime());
            telemetry.addData("HangerState:",hangerState);
            telemetry.addData("Hook Power:",rightHook.getPower());
            telemetry.update();
 ///////Drive Code////////////


            if(gamepad1.left_bumper){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y/4,
                                -gamepad1.left_stick_x/4,
                                -gamepad1.right_stick_x/4
                        )
                );
            }else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }
            drive.update();

//Code to reset everything back to base position starts

            if(gamepad1.left_bumper && gamepad1.right_bumper) {
                clawState = ATE.ClawState.CATCH;
                clawArmState = ATE.ClawArmState.BASE;
                clawWristState = ATE.ClawWristState.BASE;
                intakeWheelsState = ATE.IntakeWheelsState.HALT;
                intakeWristState = ATE.IntakeWristState.BASE;
                hSliderState = ATE.HorizontalSliderState.BASE;
                vSliderState = ATE.VerticalSliderState.BASE;
                sampleSensorState = ATE.SampleSensorState.NONE;
                clawWrist.setPosition(clawWristDropPose);
                clawArm.setPosition(clawArmDropPose);
                sleep(500);
                claw.setPosition(clawReleasePose);
                sleep(500);
                claw.setPosition(clawCatchTightPose);
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                intakeLW.setPosition(intakeWheelHaltPose);
                intakeRW.setPosition(intakeWheelHaltPose);
                intakeWrist.setPosition(intakeWristBasePose);


                vSliderTimer = resetTimer();
                vSliderTimer = startTimer();

                while((!vtSensor.isPressed()) && elapsedTime(vSliderTimer) <= ATC.vSliderMaxTime){
                    vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    telemetry.addData("MaxTimer", ATC.vSliderMaxTime);
                    telemetry.addData("VTimer", elapsedTime(vSliderTimer));
                    telemetry.update();
                    vSlider.setPower(-1);
                }
                vSlider.setPower(0);
                vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                hSliderTimer = resetTimer();
                hSliderTimer = startTimer();
                while((!htSensor.isPressed()) && elapsedTime(hSliderTimer) <= ATC.hSliderMaxTime){
                    hSliderState = ATE.HorizontalSliderState.BASE;
                    intakeWristState = ATE.IntakeWristState.CONSUME;
                    hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hSlider.setPower(-1);
                }
                hSlider.setPower(0);
                hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
//Code to reset everything back to base position ends
        }
    }
    private double startTimer() {
        return (double) System.currentTimeMillis();
    }
    private double resetTimer() {
        return 0.0;
    }
    private double elapsedTime(double startTime) {
        long now = System.currentTimeMillis();
        return ((double) now - startTime) / 1000.0;
    }

    private void setSlider(DcMotor slider, int targetPose, int velocity) {
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setTargetPosition(targetPose);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) slider).setVelocity(velocity);
    }
    private ATE.SampleSensorState getSampleSensorState(){
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
