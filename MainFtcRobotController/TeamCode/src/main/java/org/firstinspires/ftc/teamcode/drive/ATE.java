package org.firstinspires.ftc.teamcode.drive;

/*
Enums used by AT24 - Into The Deep
 */

public class ATE {

    public enum HorizontalSliderState {
        BASE,
        EXTENDED,
        MIN,
        MID,
        MAX
    };
    public enum VerticalSliderState {
        BASE,
        EXTENDED,
        SUBM_LOW,
        SUBM_HIGH,
        BASK_LOW,
        BASK_HIGH
    };
    public enum IntakeWristState {
        BASE,
        INTAKE,
        PICK_INTAKE,
        OUTTAKE,
        CONSUME
    };
    public enum IntakeWheelsState {
        HALT,
        INTAKE,
        OUTTAKE
    };
    public enum ClawState {
        CATCH,
        RELEASE
    };
    public enum ClawArmState {
        BASE,
        PICK_INTAKE,
        PICK_WALL,
        HANG,
        DROP
    };
    public enum ClawWristState {
        BASE,
        PICK_INTAKE,
        PICK_WALL,
        HANG,
        DROP
    };
    public enum SampleSensorState {
        NONE,
        BLUE,
        RED,
        YELLOW
    };
}
