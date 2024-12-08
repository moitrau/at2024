package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ATCommons;
import org.firstinspires.ftc.teamcode.CalibrationFromLimeLight;
import org.firstinspires.ftc.teamcode.drive.ATC;
import org.firstinspires.ftc.teamcode.drive.ATE;
import org.firstinspires.ftc.teamcode.drive.ATRoboTimer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(name="01_TeleopBlueAllianceStates")
public class TeleopBlueAllianceStates extends LinearOpMode {


    ATE.ClawState clawState = ATE.ClawState.CATCH;
    ATE.ClawArmState clawArmState = ATE.ClawArmState.BASE;
    ATE.ClawWristState clawWristState = ATE.ClawWristState.BASE;
    ATE.IntakeWheelsState intakeWheelsState = ATE.IntakeWheelsState.HALT;
    ATE.IntakeWristState intakeWristState = ATE.IntakeWristState.BASE;
    ATE.HorizontalSliderState hSliderState = ATE.HorizontalSliderState.BASE;
    ATE.VerticalSliderState vSliderState = ATE.VerticalSliderState.BASE;
    ATE.SampleSensorState sampleSensorState = ATE.SampleSensorState.NONE;
    ATE.SampleSensorState presampleSensorState = ATE.SampleSensorState.NONE;
    ATE.HangerState hangerState = ATE.HangerState.IDLE;
    ATE.OrcaModeState orcaModeState = ATE.OrcaModeState.DISABLED;

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
    private ColorSensor presampleSensor;
    private TouchSensor vtSensor;
    private TouchSensor htSensor;
    public DistanceSensor distanceSensor;
    private LED led0;
    private LED led1;
    private LED led2;
    private LED led3;
    private Limelight3A limelight;
    private CalibrationFromLimeLight calibrationFromLimeLight;

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
    int vSliderCurrentPose=0;

    int hSliderBasePose = ATC.hSliderBasePose;
    int hSliderMinPose = ATC.hSliderMinPose;
    int hSliderMaxPose = ATC.hSliderMaxPose;
    int hSliderVelocity = ATC.hSliderVelocity;

    private double intakeWristTimer = 0;
    private double hSliderTimer = 0;
    private double vSliderTimerOld = 0;


    ATRoboTimer consumeTimer = new ATRoboTimer();
    ATRoboTimer clawTimer = new ATRoboTimer();
    ATRoboTimer grabTimer = new ATRoboTimer();
    ATRoboTimer pickTimer = new ATRoboTimer();
    ATRoboTimer dropTimer = new ATRoboTimer();
    ATRoboTimer intakeTimer = new ATRoboTimer();
    ATRoboTimer orcaModeTimer = new ATRoboTimer();
    ATRoboTimer vSliderTimer = new ATRoboTimer();
    private ATCommons atCommons;

    double[] llPose;
    double rrllDeltaY = 0.0;
    double rrllDeltaHeading = 0.0;

    boolean ledBlinkerVar = false;

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
        presampleSensor = hardwareMap.get(ColorSensor.class, "precolorSensor");
        led0 = hardwareMap.get(LED.class, "led0");
        led1 = hardwareMap.get(LED.class, "led1");
        led2 = hardwareMap.get(LED.class, "led2");
        led3 = hardwareMap.get(LED.class, "led3");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        atCommons = new ATCommons();
        atCommons.init(hardwareMap);
//Set Drive Mode
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//Initialize Poses

        intakeWrist.setPosition(intakeWristConsumePose);
        claw.setPosition(clawCatchTightPose);
        intakeLW.setPosition(intakeWheelHaltPose);
        intakeRW.setPosition(intakeWheelHaltPose);
        clawArm.setPosition(clawArmBasePose);
        sleep(1000);
        intakeWrist.setPosition(intakeWristBasePose);
        clawWrist.setPosition(clawWristBasePose);
        setLedLights(false);
        vSliderTimerOld = resetTimer();
        vSliderTimerOld = startTimer();

        vSlider.setDirection(DcMotor.Direction.REVERSE);
        while((!vtSensor.isPressed()) && elapsedTime(vSliderTimerOld) <= ATC.vSliderMaxTime){
            vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("MaxTimer", ATC.vSliderMaxTime);
            telemetry.addData("VTimer", elapsedTime(vSliderTimerOld));
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
                if(!gamepad1.x && ! gamepad1.y) {
                    intakeWrist.setPosition(ATC.intakeWristBasePose);
                }
                hSlider.setPower(-1);
                //Below if condition is to extend horizontal slider outside
            }else if(gamepad1.dpad_right && clawWristState != ATE.ClawWristState.PICK_WALL && hSlider.getCurrentPosition()<=ATC.hSliderMaxPose){
                hSliderState = ATE.HorizontalSliderState.EXTENDED;
                intakeWristState = ATE.IntakeWristState.BASE;
                if(!gamepad1.x && ! gamepad1.y) {
                    intakeWrist.setPosition(ATC.intakeWristBasePose);
                }
                hSlider.setPower(0.5);
                //If no buttons are pressed move intake wrist to base position
            }else if(hangerState == ATE.HangerState.IDLE){
                //intakeWrist.setPosition(ATC.intakeWristBasePose);
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
                if(vSliderState != ATE.VerticalSliderState.BASE ){//vSliderState == ATE.VerticalSliderState.EXTENDED || vSliderState == ATE.VerticalSliderState.SUBM_HIGH || vSliderState == ATE.VerticalSliderState.SUBM_LOW ) {
                    vSliderState = ATE.VerticalSliderState.BASE;
                    vSlider.setPower(0);
                    vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

//Intake-Outtake code for sample starts
            //Below code is to start outtake
            if ( ((gamepad1.y && hSlider.getCurrentPosition() >= ATC.hSliderMinPose) || sampleSensorState == ATE.SampleSensorState.RED || presampleSensorState == ATE.SampleSensorState.RED || (sampleSensorState != ATE.SampleSensorState.NONE && presampleSensorState != ATE.SampleSensorState.NONE )) && hSliderState == ATE.HorizontalSliderState.EXTENDED ) {
                intakeLW.setDirection(Servo.Direction.FORWARD);
                intakeRW.setDirection(Servo.Direction.REVERSE);
                intakeLW.setPosition(0.9);
                intakeRW.setPosition(0.9);
                sampleSensorState = getSampleSensorState(sampleSensor);
                intakeWrist.setPosition(ATC.intakeWristOuttakePose);
                //Below if condition is to start intake
            }else if ((gamepad1.x && hSlider.getCurrentPosition() >= ATC.hSliderMinPose) && sampleSensorState == ATE.SampleSensorState.NONE && hSliderState == ATE.HorizontalSliderState.EXTENDED) {
                intakeLW.setDirection(Servo.Direction.REVERSE);
                intakeRW.setDirection(Servo.Direction.FORWARD);
                intakeLW.setPosition(0.9);
                intakeRW.setPosition(0.9);
                intakeWrist.setPosition(ATC.intakeWristIntakePose);
            }else if ((sampleSensorState == ATE.SampleSensorState.YELLOW || sampleSensorState == ATE.SampleSensorState.BLUE ) && hSliderState == ATE.HorizontalSliderState.EXTENDED){
                hSlider.setPower(-0.8);
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                clawWrist.setPosition(clawWristBasePose);
                clawArm.setPosition(clawArmIntakePose);
                claw.setPosition(clawReleasePose);
                intakeWrist.setPosition(ATC.intakeWristBasePose);
            }else if( sampleSensorState == ATE.SampleSensorState.NONE && hSliderState == ATE.HorizontalSliderState.EXTENDED){
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                intakeWrist.setPosition(ATC.intakeWristBasePose);
            }

//Sample sensor code starts
            if(sampleSensorState == ATE.SampleSensorState.NONE) {
                sampleSensorState = getSampleSensorState(sampleSensor);
            }

            presampleSensorState = getPreSampleSensorState(presampleSensor);

//Sample sensor code ends
            if(clawArmState == ATE.ClawArmState.PICK_INTAKE && clawWristState == ATE.ClawWristState.PICK_INTAKE && intakeWristState == ATE.IntakeWristState.PICK_INTAKE ) {
                if(consumeTimer.isActive) {
                    intakeLW.setDirection(Servo.Direction.FORWARD);
                    intakeRW.setDirection(Servo.Direction.REVERSE);
                    intakeLW.setPosition(0.9);
                    intakeRW.setPosition(0.9);
                    intakeWrist.setPosition(ATC.intakeWristConsumePose);
                    if (consumeTimer.elapsedTime() >= ATC.consumeMaxTime){
                        consumeTimer.stopTimer();
                        //grabTimer.startTimer();
                        clawTimer.startTimer();
                    }
                }
                /*if(grabTimer.isActive) {
                    intakeWrist.setPosition(ATC.intakeWristPickIntakePose);
                    clawWrist.setPosition(clawWristBasePose);
                    clawArm.setPosition(clawArmIntakePose);
                    claw.setPosition(clawReleasePose);

                    if (grabTimer.elapsedTime() >= ATC.grabMaxTime){
                        grabTimer.stopTimer();
                        clawTimer.startTimer();
                    }
                }*/
                if (clawTimer.isActive) {
                    intakeWrist.setPosition(ATC.intakeWristPickIntakePose);
                    clawWrist.setPosition(clawWristIntakePose);
                    claw.setPosition(clawCatchLoosePose);
                    if (clawTimer.elapsedTime() >= ATC.clawMaxTime){
                        clawTimer.stopTimer();
                        pickTimer.startTimer();
                    }
                }
                if(pickTimer.isActive){
                    clawArm.setPosition(clawArmBasePose);
                    intakeLW.setPosition(0.5);
                    intakeRW.setPosition(0.5);
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

            if(gamepad1.a && hSliderState != ATE.HorizontalSliderState.EXTENDED && sampleSensorState == ATE.SampleSensorState.NONE  && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE ){
                clawArm.setPosition(ATC.clawArmWallPose);
                clawWrist.setPosition(ATC.clawWristWallPose);
                sleep(250);
                claw.setPosition(clawReleasePose);
                clawWristState = ATE.ClawWristState.PICK_WALL;
            }

            if(gamepad1.dpad_up && gamepad1.left_bumper ){
                claw.setPosition(clawCatchTightPose);
                clawWrist.setPosition(clawWristHangPose);
                clawArm.setPosition(clawArmHangPose);
                setSlider(vSlider,ATC.vSliderSpecimenSwipePose,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.SPECIMEN_SWIPE;
            }
            if(gamepad1.dpad_down && gamepad1.left_bumper){
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                clawWristState = ATE.ClawWristState.BASE;
                clawArmState = ATE.ClawArmState.BASE;
                vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vSliderState = ATE.VerticalSliderState.EXTENDED;
                vSlider.setPower(-1);
            }



            if(gamepad1.b && sampleSensorState == ATE.SampleSensorState.NONE && ( clawWristState == ATE.ClawWristState.PICK_FLOOR || clawWristState == ATE.ClawWristState.PICK_WALL ) && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE ){
                claw.setPosition(clawCatchTightPose);
                sleep(500);
                clawWrist.setPosition(clawWristBasePose);
                clawArm.setPosition(clawArmBasePose);
                setSlider(vSlider,ATC.vSliderWallLiftPose,vSliderVelocity);
                clawWristState = ATE.ClawWristState.BASE;

            }
            
            if(gamepad2.a && sampleSensorState == ATE.SampleSensorState.NONE && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE ){
                claw.setPosition(clawCatchTightPose);
                clawWrist.setPosition(clawWristHangPose);
                clawArm.setPosition(clawArmHangPose);
                setSlider(vSlider,vSliderSubmHighPose+50,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.SUBM_HIGH;
                clawWristState = ATE.ClawWristState.HANG;
            }


            if(gamepad2.b && sampleSensorState == ATE.SampleSensorState.NONE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.SUBM_HIGH ){
                atCommons.selfAdjust(ATC.submAdjustDistance,ATC.submAdjustMaxTime,ATC.submAdjustPower);
                claw.setPosition(clawCatchTightPose);
                clawWrist.setPosition(clawWristHangPose);
                clawArm.setPosition(clawArmHangPose);
                setSlider(vSlider,ATC.vSliderSubmPullUpPose+50,10000);
                vSliderState = ATE.VerticalSliderState.SUBM_PULLUP;
                //clawWrist.getController().pwmDisable();
                clawArmState = ATE.ClawArmState.HANG;
            }
/*
            if(gamepad2.left_trigger > 0.7 && orcaModeState == ATE.OrcaModeState.DISABLED) {
                orcaModeState = ATE.OrcaModeState.CLIP_SPECIMEN_TO_SUBM;
                orcaModeTimer.startTimer();
            }
            if (orcaModeTimer.isActive) {
                if (orcaModeTimer.elapsedTime() >= ATC.orcaModeMaxTime){
                    orcaModeTimer.stopTimer();
                    setLedLights(false);
                    orcaModeState = ATE.OrcaModeState.DISABLED;
                }
            }
            if(orcaModeState == ATE.OrcaModeState.CLIP_SPECIMEN_TO_SUBM && orcaModeTimer.elapsedTime() <= ATC.orcaModeMaxTime){

                calibrationFromLimeLight = new CalibrationFromLimeLight();
                calibrationFromLimeLight.LimeLight3A(0, 0, 0, 0.5, hardwareMap);

                llPose = calibrationFromLimeLight.CalibratePoseWithLimelIght();
                ledBlinkerVar = !ledBlinkerVar;
                setLedLights(ledBlinkerVar);
                if(llPose[0] > -10.0 && llPose[0] < 10.0 && llPose[1] > 48.0  && rrllDeltaY != 0.0) {
                    setLedLights(true);
                    Pose2d correctedPose = new Pose2d(drive.getPoseEstimate().getX(), llPose[1] + rrllDeltaY, Math.toRadians(llPose[2] + rrllDeltaHeading));
                    drive.setPoseEstimate(correctedPose);
                    setSlider(vSlider, vSliderSubmHighPose + 50, vSliderVelocity);
                    claw.setPosition(clawCatchTightPose);
                    clawWrist.setPosition(clawWristHangPose);
                    clawArm.setPosition(clawArmHangPose);
                    vSliderTimer.startTimer();
                    TrajectorySequence trajSequence = drive.trajectorySequenceBuilder(correctedPose)
                            .setReversed(false)
                            .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), 37, Math.toRadians(90)))
                            .build();
                    drive.followTrajectorySequence(trajSequence);

                    while (vSlider.getCurrentPosition() < ATC.vSliderSubmHighPose-10) {
                        if (vSliderTimer.isActive) {
                            if (vSliderTimer.elapsedTime() >= 3){
                                vSliderTimer.stopTimer();
                                break;
                            }
                        }
                    }
                    claw.setPosition(clawCatchTightPose);
                    clawWrist.setPosition(clawWristHangPose);
                    clawArm.setPosition(clawArmHangPose);
                    setSlider(vSlider, ATC.vSliderSubmLowPose, 750);
                    vSliderState = ATE.VerticalSliderState.SUBM_LOW;
                    clawArmState = ATE.ClawArmState.HANG;
                    setLedLights(false);
                    orcaModeState = ATE.OrcaModeState.DISABLED;
                }
            }
            if(gamepad2.right_trigger > 0.7 ){
                ///drive.breakFollowing();
            }

            if(gamepad1.left_trigger > 0.7 && drive.getPoseEstimate().getY() > 39 && drive.getPoseEstimate().getX() < -45 ){
                setLedLights(true);
                claw.setPosition(clawCatchTightPose);
                sleep(500);
                clawWrist.setPosition(clawWristBasePose);
                clawArm.setPosition(clawArmBasePose);
                setSlider(vSlider,ATC.vSliderWallLiftPose,vSliderVelocity);
                clawWristState = ATE.ClawWristState.BASE;
                TrajectorySequence trajSequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0,46,Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSequence);

            }
            if(gamepad1.right_trigger > 0.7 && drive.getPoseEstimate().getX() > -12 && drive.getPoseEstimate().getX() < 12 && drive.getPoseEstimate().getY() > 35 && drive.getPoseEstimate().getY() < 44  ){
                setLedLights(true);
                TrajectorySequence trajSequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-50,42,Math.toRadians(-90)),
                                SampleMecanumDrive.getVelocityConstraint(60.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(40, () -> {
                            clawArm.setPosition(ATC.clawArmFloorPose);
                            clawWrist.setPosition(ATC.clawWristFloorPose);
                            claw.setPosition(clawReleasePose);
                            clawWristState = ATE.ClawWristState.PICK_FLOOR;
                        })
                        .build();
                drive.followTrajectorySequence(trajSequence);
                setLedLights(false);

            }
*/

           /* if( vSlider.getCurrentPosition() <= ATC.vSliderSubmHighPose-200  && sampleSensorState == ATE.SampleSensorState.NONE  && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.SUBM_LOW ){
                clawArm.setPosition(clawArmWallPose);
                clawWrist.setPosition(clawWristDropPose);

            }
            
            if( vSlider.getCurrentPosition() <= ATC.vSliderSubmHighPose-300 && sampleSensorState == ATE.SampleSensorState.NONE  && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.SUBM_LOW ){
                claw.setPosition(clawReleasePose);
                vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vSliderState = ATE.VerticalSliderState.EXTENDED;
                vSlider.setPower(-1);
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                clawWristState = ATE.ClawWristState.BASE;
                clawArmState = ATE.ClawArmState.BASE;

            } */
            if( vSlider.getCurrentPosition() >= ATC.vSliderSubmPullUpPose-20 && sampleSensorState == ATE.SampleSensorState.NONE  && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.SUBM_PULLUP ){
                claw.setPosition(clawReleasePose);
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vSliderState = ATE.VerticalSliderState.EXTENDED;
                vSlider.setPower(-1);

                clawWristState = ATE.ClawWristState.BASE;
                clawArmState = ATE.ClawArmState.BASE;

            }


//Specimen code ends

//Code to hang onto submersible starts
            if (gamepad2.dpad_up && gamepad2.right_bumper && hangerState == ATE.HangerState.IDLE) {
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

            if (gamepad2.dpad_up && gamepad2.left_bumper && hangerState == ATE.HangerState.ACTIVE) {
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

            telemetry.addData("Roadrunner X",drive.getPoseEstimate().getX());
            telemetry.addData("Roadrunner Y",drive.getPoseEstimate().getY());
            telemetry.addData("Roadrunner Heading",Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("claw state", orcaModeState);
            telemetry.addData("claw state", clawState);
            telemetry.addData("clawWrist state", clawWristState);
            telemetry.addData("clawArm state", clawArmState);
            telemetry.addData("intakeWrist state", intakeWristState);
            telemetry.addData("intakeWheels state", intakeWheelsState);
            telemetry.addData("sample sensor state", sampleSensorState);
            telemetry.addData("presample sensor state", presampleSensorState);
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

            ((NormalizedColorSensor) sampleSensor).setGain(2);
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance:",distance);
            double sampdistance = ((DistanceSensor) sampleSensor).getDistance(DistanceUnit.CM);
            double hue = Double.parseDouble(JavaUtil.formatNumber(JavaUtil.colorToHue(((NormalizedColorSensor) sampleSensor).getNormalizedColors().toColor()), 0));
            telemetry.addData("Sample Sensor Hue:",hue);
            telemetry.addData("Sample Sensor Distance:",sampdistance);
            ((NormalizedColorSensor) presampleSensor).setGain(2);
            double presampdistance = ((DistanceSensor) presampleSensor).getDistance(DistanceUnit.CM);
            double prehue = Double.parseDouble(JavaUtil.formatNumber(JavaUtil.colorToHue(((NormalizedColorSensor) presampleSensor).getNormalizedColors().toColor()), 0));
            telemetry.addData("PreSample Sensor Hue:",prehue);
            telemetry.addData("PreSample Sensor Distance:",presampdistance);
            telemetry.update();
            ///////Drive Code////////////

            if (vSliderState == ATE.VerticalSliderState.SUBM_HIGH && distance <= ATC.speciPickAdjustDistance && gamepad1.left_stick_y > 0){
////Do nothing
            } else if(gamepad1.left_bumper ||  vSliderState == ATE.VerticalSliderState.SUBM_HIGH || clawWristState == ATE.ClawWristState.PICK_FLOOR || clawWristState == ATE.ClawWristState.PICK_WALL){
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
                if (hSliderState == ATE.HorizontalSliderState.EXTENDED){
                    setSlider(hSlider, 1400, hSliderVelocity);
                }

                clawState = ATE.ClawState.CATCH;
                clawArmState = ATE.ClawArmState.BASE;
                clawWristState = ATE.ClawWristState.BASE;
                intakeWheelsState = ATE.IntakeWheelsState.HALT;
                intakeWristState = ATE.IntakeWristState.BASE;
                hSliderState = ATE.HorizontalSliderState.BASE;
                vSliderState = ATE.VerticalSliderState.BASE;
                sampleSensorState = ATE.SampleSensorState.NONE;
                //presampleSensorState = ATE.SampleSensorState.NONE;

                intakeLW.setPosition(intakeWheelHaltPose);
                intakeRW.setPosition(intakeWheelHaltPose);
                intakeWrist.setPosition(intakeWristBasePose);

                vSliderTimerOld = resetTimer();
                vSliderTimerOld = startTimer();

                while((!vtSensor.isPressed()) && elapsedTime(vSliderTimerOld) <= ATC.vSliderMaxTime){
                    vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    telemetry.addData("MaxTimer", ATC.vSliderMaxTime);
                    telemetry.addData("VTimer", elapsedTime(vSliderTimerOld));
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
                clawWrist.setPosition(clawWristDropPose);
                clawArm.setPosition(clawArmDropPose);
                sleep(500);
                claw.setPosition(clawReleasePose);
                sleep(500);
                claw.setPosition(clawCatchTightPose);
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
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
    private ATE.SampleSensorState getSampleSensorState(ColorSensor colSensor){
        ((NormalizedColorSensor) colSensor).setGain(2);
        double distance = ((DistanceSensor) colSensor).getDistance(DistanceUnit.CM);

        double hue = Double.parseDouble(JavaUtil.formatNumber(JavaUtil.colorToHue(((NormalizedColorSensor) colSensor).getNormalizedColors().toColor()), 0));

        if (distance <= 4 && hue >= 200 && hue <= 250) {
            return ATE.SampleSensorState.BLUE;
        } else if (distance <= 4 && hue >= 65 && hue <= 100) {
            return ATE.SampleSensorState.YELLOW;
        } else if (distance <= 4 && hue >= 0 && hue <= 60) {
            return ATE.SampleSensorState.RED;
        } else {
            return ATE.SampleSensorState.NONE;
        }
    }
    private ATE.SampleSensorState getPreSampleSensorState(ColorSensor colSensor){

        ((NormalizedColorSensor) colSensor).setGain(2);
        double distance = ((DistanceSensor) colSensor).getDistance(DistanceUnit.CM);

        double hue = Double.parseDouble(JavaUtil.formatNumber(JavaUtil.colorToHue(((NormalizedColorSensor) colSensor).getNormalizedColors().toColor()), 0));

        if (distance <= 5.5 && hue >= 200 && hue <= 250) {
            return ATE.SampleSensorState.BLUE;
        } else if (distance <= 5.5 && hue >= 41 && hue <= 100) {
            return ATE.SampleSensorState.YELLOW;
        } else if (distance <= 5.5 && hue >= 0 && hue <= 40) {
            return ATE.SampleSensorState.RED;
        } else {
            return ATE.SampleSensorState.NONE;
        }
    }
    private void setLedLights(boolean switchOn){
        led0.enable(switchOn);
        led1.enable(switchOn);
        led2.enable(switchOn);
        led3.enable(switchOn);
    }

}
