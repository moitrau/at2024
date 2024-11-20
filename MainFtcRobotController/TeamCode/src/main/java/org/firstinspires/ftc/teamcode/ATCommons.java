package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.ATE;
import org.firstinspires.ftc.teamcode.drive.ATRoboTimer;


public class ATCommons {

    public Limelight3A limelight;
    public double xCordinate;
    public double yCordinate;
    public double yawAngleRobot;
    public double marginOfError = 0.5;
    public double resultCordinates[] = new double[3];
    HardwareMap hardwareMap;
    public DcMotor rightRear;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor leftFront;
    public DcMotor hSlider;
    public DcMotor vSlider;
    public DcMotor rightHook;
    public DcMotor leftHook;
    public Servo clawArm;
    public Servo clawWrist;
    public Servo claw;
    public Servo intakeWrist;
    public Servo intakeLW;
    public Servo intakeRW;
    public ColorSensor sampleSensor;
    public DistanceSensor distanceSensor;
    public TouchSensor vtSensor;
    public TouchSensor htSensor;

    public void init(HardwareMap hMap)
    {
        hardwareMap = hMap;
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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

    }
    public void selfAdjust(double distance, double maxTime, double power){
        ATRoboTimer selfAdjustTimer = new ATRoboTimer();
        selfAdjustTimer.startTimer();
        while(distanceSensor.getDistance(DistanceUnit.CM)>=distance && selfAdjustTimer.elapsedTime() <= maxTime) {
            rightFront.setPower(power);
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);
        }
        selfAdjustTimer.stopTimer();
        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    public void startSelfAdjustAsync(double distance, double power){
        ATRoboTimer selfAdjustTimer = new ATRoboTimer();
        selfAdjustTimer.startTimer();
        if(distanceSensor.getDistance(DistanceUnit.CM)>=distance) {
            rightFront.setPower(power);
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);
        }else{
            rightFront.setPower(0);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }
    }
    public void stopSelfAdjustAsync(){

        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }
    public ATE.SampleSensorState getSampleSensorState(ColorSensor sampleSensor) {
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
