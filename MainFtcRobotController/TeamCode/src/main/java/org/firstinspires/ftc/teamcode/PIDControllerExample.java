import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;
@Disabled
@TeleOp(name = "PIDControllerExample")
public class PIDControllerExample extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    private List<DcMotorEx> motors;

    private double kP = 0.01; // Proportional gain
    private double kI = 0.01; // Integral gain
    private double kD = 0.01; // Derivative gain

    private double targetPosition = 10000; // Target position in encoder ticks
    private double targetVelocity = 5; // Target velocity in encoder ticks per second

    private double previousError = 0;
    private double integral = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Read encoder values
            double currentPosition = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 2.0;
            double currentVelocity = (frontLeft.getVelocity() + frontRight.getVelocity()) / 2.0;

            // Calculate PID output
            double output = calculatePID(currentPosition, currentVelocity);


            // Set motor power
            frontLeft.setPower(output);
            frontRight.setPower(output);

                telemetry.addData("velocityKey", frontLeft.getVelocity());
                telemetry.addData("CurrentPosition", frontLeft.getCurrentPosition());
                telemetry.addData("TargerPosition", targetPosition);
                telemetry.addData("output", output);
                telemetry.addData("isAtTargetKey", !frontLeft.isBusy());
                telemetry.update();


            // Add a small delay for stability
            sleep(50);

        }
    }

    private double calculatePID(double currentPosition, double currentVelocity) {
        double positionError = targetPosition - currentPosition;
        double velocityError = targetVelocity - currentVelocity;

        // Proportional term
        double pTerm = kP * positionError;

        // Integral term
        integral += positionError;
        double iTerm = kI * integral;

        // Derivative term
        double dTerm = kD * (positionError - previousError);
        previousError = positionError;

        // Combine PID terms
        return pTerm + iTerm + dTerm;
    }
}
