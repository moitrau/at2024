package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PranavEncoder From Android")
public class PranavEncoderJava extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    /**
     * nothing to see here
     */
    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        for (int count = 0; count < 1; count++) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            rearLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            rearRight.setDirection(DcMotor.Direction.FORWARD);
            waitForStart();
            if (opModeIsActive()) {
                frontLeft.setTargetPosition(5377);
                frontRight.setTargetPosition(5377);
                rearLeft.setTargetPosition(5377);
                rearRight.setTargetPosition(5377);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) frontLeft).setVelocity(1075.4);
                ((DcMotorEx) frontRight).setVelocity(1075.4);
                ((DcMotorEx) rearLeft).setVelocity(1075.4);
                ((DcMotorEx) rearRight).setVelocity(1075.4);
                while (frontLeft.isBusy() && !isStopRequested()) {
                    telemetry.addData("velocityKey", ((DcMotorEx) frontLeft).getVelocity());
                    telemetry.addData("positionKey", frontLeft.getCurrentPosition());
                    telemetry.addData("isAtTargetKey", !frontLeft.isBusy());
                    telemetry.update();
                }
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                rearLeft.setDirection(DcMotor.Direction.FORWARD);
                rearRight.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setTargetPosition(5377);
                frontRight.setTargetPosition(5377);
                rearLeft.setTargetPosition(5377);
                rearRight.setTargetPosition(5377);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) frontLeft).setVelocity(1075.4);
                ((DcMotorEx) frontRight).setVelocity(1075.4);
                ((DcMotorEx) rearLeft).setVelocity(1075.4);
                ((DcMotorEx) rearRight).setVelocity(1075.4);
                while (frontLeft.isBusy() && !isStopRequested()) {
                    telemetry.addData("velocityKey", ((DcMotorEx) frontLeft).getVelocity());
                    telemetry.addData("positionKey", frontLeft.getCurrentPosition());
                    telemetry.addData("isAtTargetKey", !frontLeft.isBusy());
                    telemetry.update();
                }
            }
        }
    }
}
