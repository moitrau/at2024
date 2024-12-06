package org.firstinspires.ftc.teamcode.drive;

/*
Constants used by AT24 - Into The Deep
 */
public class ATC {

    public static final double clawCatchTightPose = 1;
    public static final double clawCatchLoosePose = 0.7;
    public static final double clawReleasePose = 0.4;

    public static final double clawWristBasePose = 0.60;
    public static final double clawWristIntakePose = 0.70;
    public static final double clawWristHangPose = 0.05;
    public static final double clawWristDropPose = 0.30;
    public static final double clawWristWallPose = 0.08;
    public static final double clawWristFloorPose = 0.30;

    public static final double clawArmBasePose = 0.55;
    public static final double clawArmIntakePose = 0.95;
    public static final double clawArmHangPose = 0.58;
    public static final double clawArmDropPose = 0.40;
    public static final double clawArmWallPose = 0.47;
    public static final double clawArmFloorPose = 0.10;

    public static final double intakeWristBasePose = 0.32;
    public static final double intakeWristPickIntakePose = 0.25;
    public static final double intakeWristConsumePose = 0.0;
    public static final double intakeWristOuttakePose = 0.46;
    public static final double intakeWristIntakePose = 0.7;
    //public static final double intakeWristIntakePose = 0.6;
    public static final double intakeWheelHaltPose = 0.5;
    public static final double intakeWheelRunPose = 0.9;


    public static final int vSliderBasePose = 0;
    public static final int vSliderWallLiftPose = 375;
    public static final int vSliderSubmHighPose = 1150;
    public static final int vSliderSubmPullUpPose = 1600;
    public static final int vSliderSubmMidPose = 900;
    public static final int vSliderSubmLowPose = 675;
    public static final int vSliderBaskHighPose = 2800;
    //public static final int vSliderBaskHighPose = 1750;
    public static final int vSliderBaskLowPose = 1400;
    public static final int vSliderSpecimenSwipePose = 900;
    public static final int vSliderVelocity=10000;
    public static final int vSliderHangVelocity=10000;

    //Stage1 diff determines when to push the arm while going down
    public static final int submHangStage1Diff = 300;
    //Stage2 diff determines when to release the claw and reset the arm
    public static final int submHangStage2Diff = 400;

    public static final int hSliderBasePose = 0;
    public static final int hSliderMinPose = 470;
    public static final int hSliderMaxPose = 1700;
    public static final int hSliderVelocity=10000;

    public static final double intakeWristMaxTime = 1.0;
    public static final double hSliderMaxTime = 10.0;
    public static final double vSliderMaxTime = 10.0;
    public static final double consumeMaxTime = 1.0;
    public static final double grabMaxTime = 0.75;
    public static final double clawMaxTime = 0.5;
    public static final double pickMaxTime = 0.5;
    public static final double dropMaxTime = 0.5;
    public static final double outtakeMaxTime = 2.0;
    public static final double orcaModeMaxTime = 3.0;
    public static final double submAdjustMaxTime = 5;
    public static final double wallAdjustMaxTime = 5;
    public static final double speciPickAdjustMaxTime = 3.0;
    //Below values are in cms
    public static final double submAdjustDistance = 10.0;
    public static final double wallAdjustDistance = 10.0;
    public static final double speciPickAdjustDistance = 14.3;
    //Power
    public static final double submAdjustPower = -0.25;
    public static final double wallAdjustPower = -0.25;
    public static final double speciPickAdjustPower = -0.25;
}
