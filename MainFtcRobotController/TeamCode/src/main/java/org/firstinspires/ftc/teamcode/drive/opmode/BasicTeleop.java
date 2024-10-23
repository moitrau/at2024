package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="BasicTeleop2")
public class BasicTeleop extends LinearOpMode {


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

    double clawCatchPose = ATC.clawCatchPose;
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
    private double outtakeTimer = 0;

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
        claw.setPosition(clawCatchPose);
        clawArm.setPosition(clawArmBasePose);
        clawWrist.setPosition(clawArmBasePose);
        intakeLW.setPosition(intakeWheelHaltPose);
        intakeRW.setPosition(intakeWheelHaltPose);
        intakeWrist.setPosition(intakeWristBasePose);
        sleep(2000);
        telemetry.addData("TouchSensor Check",1);
        telemetry.update();

        vSliderTimer = resetTimer();
        vSliderTimer = startTimer();

        while((!vtSensor.isPressed()) && elapsedTime(vSliderTimer) <= ATC.vSliderMaxTime){
            vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("MaxTimer", ATC.vSliderMaxTime);
            telemetry.addData("VTimer", elapsedTime(vSliderTimer));
            telemetry.update();
            vSlider.setPower(-0.5);
        }
        vSlider.setPower(0);
        vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hSliderTimer = resetTimer();
        hSliderTimer = startTimer();
        while((!htSensor.isPressed()) && elapsedTime(hSliderTimer) <= ATC.hSliderMaxTime){
            hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hSlider.setPower(0.5);
        }
        hSlider.setPower(0);
        hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlider.setDirection(DcMotor.Direction.REVERSE);
        hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//Wait till Play button is clicked on DriverHub
        waitForStart();

        while (!isStopRequested()) {

            if(htSensor.isPressed()){
                hSliderState = ATE.HorizontalSliderState.BASE;
                hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.dpad_left && hSliderState != ATE.HorizontalSliderState.BASE) {
                telemetry.addData("Horizontal Slider:", hSlider.getCurrentPosition());
                hSlider.setPower(-0.5);
            }else if(gamepad2.dpad_right){
                hSliderState = ATE.HorizontalSliderState.EXTENDED;
                telemetry.addData("Horizontal Slider:", hSlider.getCurrentPosition());
                intakeWrist.setPosition(ATC.intakeWristBasePose);
                hSlider.setPower(0.5);
            } else {
                telemetry.addData("Zero Power Behavior", hSlider.getZeroPowerBehavior());
                hSlider.setPower(0);
            }

            if(vtSensor.isPressed()){
                vSliderState = ATE.VerticalSliderState.BASE;
                vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad2.left_bumper && gamepad2.dpad_up && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                setSlider(vSlider,vSliderBaskHighPose,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.EXTENDED;
            }

/* Code to move up the slider for hanging specimen - disabled for now
            if(gamepad2.right_bumper && gamepad2.dpad_up && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                setSlider(vSlider,vSliderSubmHighPose,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.EXTENDED;
            }
 */
            if(gamepad2.dpad_down && clawWristState == ATE.ClawWristState.BASE && clawArmState == ATE.ClawArmState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.EXTENDED ){
                setSlider(vSlider,vSliderBasePose,vSliderVelocity);
                vSliderState = ATE.VerticalSliderState.BASE;
            }

            /*
            if (gamepad2.dpad_down && vSliderState != ATE.VerticalSliderState.BASE) {
                vSlider.setPower(-0.5);
            }else if(gamepad2.dpad_up){
                vSliderState = ATE.VerticalSliderState.EXTENDED;
                vSlider.setPower(0.5);
            } else {
                telemetry.addData("Zero Power Behavior", vSlider.getZeroPowerBehavior());
                vSlider.setPower(0);
            }*/

//Intake-Outtake code for sample starts
            if ((gamepad2.left_bumper && gamepad2.y) || sampleSensorState == ATE.SampleSensorState.RED) {
                intakeLW.setDirection(Servo.Direction.FORWARD);
                intakeRW.setDirection(Servo.Direction.REVERSE);
                intakeLW.setPosition(0.9);
                intakeRW.setPosition(0.9);
                sampleSensorState = getSampleSensorState();
                intakeWrist.setPosition(ATC.intakeWristOuttakePose);
            }else if ((gamepad2.left_bumper && gamepad2.x) && sampleSensorState == ATE.SampleSensorState.NONE && hSliderState == ATE.HorizontalSliderState.EXTENDED) {
                intakeLW.setDirection(Servo.Direction.REVERSE);
                intakeRW.setDirection(Servo.Direction.FORWARD);
                intakeLW.setPosition(0.9);
                intakeRW.setPosition(0.9);
                intakeWrist.setPosition(ATC.intakeWristIntakePose);
                hSlider.setPower(0.3);
            }else if ((sampleSensorState == ATE.SampleSensorState.YELLOW || sampleSensorState == ATE.SampleSensorState.BLUE ) && hSliderState != ATE.HorizontalSliderState.BASE){
                hSlider.setPower(-0.5);
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                intakeWrist.setPosition(ATC.intakeWristBasePose);
            }else{
                intakeLW.setPosition(0.5);
                intakeRW.setPosition(0.5);
                intakeWrist.setPosition(ATC.intakeWristBasePose);
            }

//Code detect sample using color sensor
            if(sampleSensorState == ATE.SampleSensorState.NONE) {
                sampleSensorState = getSampleSensorState();
            }
            
            if(gamepad2.a && hSliderState == ATE.HorizontalSliderState.BASE && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                clawWrist.setPosition(clawWristIntakePose);
                clawArm.setPosition(clawArmIntakePose);
                claw.setPosition(clawReleasePose);
                claw.setPosition(clawCatchPose);
                clawWristState = ATE.ClawWristState.PICK_INTAKE;
            }

            if(gamepad2.b && clawWristState == ATE.ClawWristState.PICK_INTAKE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                claw.setPosition(clawCatchPose);
                clawWristState = ATE.ClawWristState.BASE;
            }
//Intake-Outtake code for sample ends



//Sample code starts

            if(gamepad2.left_bumper && gamepad2.b && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.EXTENDED && (sampleSensorState ==ATE.SampleSensorState.BLUE ||sampleSensorState ==ATE.SampleSensorState.YELLOW )){
                clawArm.setPosition(clawArmDropPose);
                clawWrist.setPosition(clawWristDropPose);
                claw.setPosition(clawReleasePose);
                clawWristState = ATE.ClawWristState.DROP;
                sampleSensorState = ATE.SampleSensorState.NONE;
            }

            if(gamepad2.left_bumper && gamepad2.a && clawWristState == ATE.ClawWristState.DROP && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.EXTENDED && sampleSensorState ==ATE.SampleSensorState.NONE ){
                clawArm.setPosition(clawArmBasePose);
                clawWrist.setPosition(clawWristBasePose);
                claw.setPosition(clawCatchPose);
                clawWristState = ATE.ClawWristState.BASE;
            }



//Sample code ends
/*
//Specimen code starts

            if(gamepad2.right_bumper && gamepad2.x && clawWristState == ATE.ClawWristState.BASE && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE ){
                clawArm.setPosition(clawArmWallPose);
                clawWrist.setPosition(clawWristWallPose);
                claw.setPosition(clawReleasePose);
                clawWristState = ATE.ClawWristState.PICK_WALL;
                sampleSensorState = ATE.SampleSensorState.NONE;
            }

            if(gamepad2.right_bumper && gamepad2.y && clawWristState == ATE.ClawWristState.PICK_WALL && hSliderState == ATE.HorizontalSliderState.BASE && vSliderState == ATE.VerticalSliderState.BASE && sampleSensorState == ATE.SampleSensorState.NONE ){
                claw.setPosition(clawCatchPose);
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
                claw.setPosition(clawCatchPose);
                clawWristState = ATE.ClawWristState.BASE;
            }
//Specimen code ends
*/

            telemetry.addData("sample sensor state", sampleSensorState);
            telemetry.addData("vslider pos", vSlider.getCurrentPosition());
            telemetry.update();
 ///////Drive Code////////////

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
/*
    private void resetSlider(DcMotor slider, TouchSensor touchSensor) {
        if (touchSensor.isPressed()) {
            slider.setPower(0);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setDirection(DcMotor.Direction.REVERSE);
        }else{
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slider.setPower(0.25);
        }
    }

    private void resetHSlider() {
        if (htSensor.isPressed()) {
            hSlider.setPower(0);
            hSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlider.setDirection(DcMotor.Direction.REVERSE);
        }else{
            hSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hSlider.setPower(0.5);
        }
    }
    private void setHSlider(int targetPose, int velocity) {
        //hSlider.setDirection(DcMotor.Direction.FORWARD);
        //slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hSlider.setTargetPosition(targetPose);
        hSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) hSlider).setVelocity(velocity);
    }
    private void setVSlider(int targetPose, int velocity) {
        //hSlider.setDirection(DcMotor.Direction.FORWARD);
        //slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlider.setTargetPosition(targetPose);
        vSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) vSlider).setVelocity(velocity);
    }
*/
    private void setSlider(DcMotor slider, int targetPose, int velocity) {
        //slider.setDirection(DcMotor.Direction.FORWARD);
        //slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
