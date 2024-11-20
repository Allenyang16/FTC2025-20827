package org.firstinspires.ftc.teamcode.uppersystems;

import static org.firstinspires.ftc.teamcode.uppersystems.SuperStructure.ClawState.GRAB;
import static org.firstinspires.ftc.teamcode.uppersystems.SuperStructure.ClawState.OPEN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Arrays;
import java.util.List;

@Config
public class SuperStructure {
    private TelemetryPacket packet = new TelemetryPacket();

    private DcMotorEx mArmLeft = null;
    private DcMotorEx mArmRight = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;
    // TODO: change kD
    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0045, 0.00011, 0.000);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients slideLeftPidConf_Horizontal = new PIDCoefficients(0.0012, 0.00, 0.00);
    private final PIDFController slideLeftPidCtrl_Horizontal;
    public static PIDCoefficients slideLeftPidConf_Vertical = new PIDCoefficients(0.0032, 0.00, 0.00);
    private final PIDFController slideLeftPidCtrl_Vertical;

    public static PIDCoefficients slideRightPidConf_Horizontal = new PIDCoefficients(0.0012, 0.00, 0.00);
    private final PIDFController slideRightPidCtrl_Horizontal;
    public static PIDCoefficients slideRightPidConf_Vertical = new PIDCoefficients(0.0032, 0.00, 0.00);
    private final PIDFController slideRightPidCtrl_Vertical;
    private List<PIDFController> slidePidCtrl;

    private Servo mClaw = null;
    private Servo mSpinWrist = null;
    private Servo mWrist = null;

    private TouchSensor armMag = null;

    public static int SLIDE_BOX_HIGH = 3200, SLIDE_BOX_LOW = 1500;
    public static int SLIDE_CHAMBER_HIGH = 1231, SLIDE_CHAMBER_LOW = 0,SLIDE_CHAMBER_DELTA = 400;
    public static int SLIDE_INTAKE_MAX = 1200, SLIDE_MIN = 0;
    // TODO: all arm values recheck
    public static int ARM_INTAKE = 1460;
    public static int ARM_POST_INTAKE = 1000;
    public static int ARM_RELEASE = -180;
    public static int ARM_CHAMBER = 50;
    // WRIST
    public static double WRIST_ORIGIN = 0.17;
    public static double WRIST_INTAKE = 0.86, WRIST_INTAKE_PARALLEL_GROUND = 0.4;
    // TODO: Retest
    public static double WRIST_RELEASE_BOX_HIGH = 0.4, WRIST_RELEASE_BOX_LOW = 0.28;
    public static double WRIST_RELEASE_CHAMBER_HIGH = 0.28, WRIST_RELEASE_CHAMBER_LOW = 0.8;

    // Spin Wrist
    public static double SPINWRIST_INTAKE = 0.29;
    public double spinwristPosition = 0.29;
    // TODO: CHANGE THE VALUE
    public static double SPINWRIST_RELEASECLIP = 0.63;
    public static double SPINWRIST_MAX = 0.46;
    public static double SPINWRIST_MIN = 0.21;
    public static double SPINWRIST_STEP = 0.08;
    public void setWristPosition(double position) {
        spinwristPosition = Math.max(SPINWRIST_MIN, Math.min(SPINWRIST_MAX, position));
        mSpinWrist.setPosition(spinwristPosition);
    }
    public double getWristPosition() {
        return spinwristPosition;
    }
    public void adjustWrist(boolean clockwise) {
        if (clockwise) {
            setWristPosition(spinwristPosition + SPINWRIST_STEP);
        } else {
            setWristPosition(spinwristPosition - SPINWRIST_STEP);
        }
    }

    // Claw
    // TODO: TEST Value
    public static double CLAW_OPEN = 0.6;
    public static double CLAW_GRAB = 0.275;
    public ClawState clawState = GRAB;
    public SlideState slideState = SlideState.VERTICAL;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure (LinearOpMode opMode){
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);

        slideLeftPidCtrl_Horizontal = new PIDFController(slideLeftPidConf_Horizontal);
        slideLeftPidCtrl_Vertical = new PIDFController(slideLeftPidConf_Vertical);

        slideRightPidCtrl_Horizontal = new PIDFController(slideRightPidConf_Horizontal);
        slideRightPidCtrl_Vertical = new PIDFController(slideRightPidConf_Vertical);

        slidePidCtrl = Arrays.asList(slideLeftPidCtrl_Horizontal,slideLeftPidCtrl_Vertical,slideRightPidCtrl_Horizontal,slideRightPidCtrl_Vertical);

        mArmLeft = hardwareMap.get(DcMotorEx.class,"armLeft");
        mArmRight = hardwareMap.get(DcMotorEx.class, "armRight");

        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mClaw = hardwareMap.get(Servo.class,"claw");
        mWrist = hardwareMap.get(Servo.class,"wrist");
        mSpinWrist = hardwareMap.get(Servo.class,"spinWrist");

//        armMag = hardwareMap.get(TouchSensor.class,"armMag");

        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mArmLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mArmRight.setDirection(DcMotorSimple.Direction.FORWARD);

        mArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // init
    public void initialize(){
        mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setSpinwristReleaseclip();
        setWristIntake();
        setClawGrab();
    }

    //update
    public void update() {
        mArmLeft.setPower(armPidCtrl.update(mArmLeft.getCurrentPosition()));
        mArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mArmRight.setPower(armPidCtrl.update(getArmRightPosition()));
        mArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(slideState == SlideState.HORIZONTAL){
            mSlideRight.setPower(slideRightPidCtrl_Horizontal.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slideLeftPidCtrl_Horizontal.update(mSlideLeft.getCurrentPosition()));
        } else if (slideState == SlideState.VERTICAL) {
            mSlideRight.setPower(slideRightPidCtrl_Vertical.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slideLeftPidCtrl_Vertical.update(mSlideLeft.getCurrentPosition()));
        }
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        packet.put("Claw State", clawState.toString());
//        packet.put("Slide State", slideState.toString());
        packet.put("rightSlide_encoder: ", getSlideRightPosition());
        packet.put("leftSlide_encoder: ", getSlideRightPosition());
        packet.put("rightArm_pos: ", getArmRightPosition());
        packet.put("leftArm_pos: ",getArmLeftPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // Claw
    public enum ClawState{
        OPEN(CLAW_OPEN),
        GRAB(CLAW_GRAB);

        private final double clawPosition;
        ClawState(double clawPosition) {
            this.clawPosition = clawPosition;
        }
        @Override
        public String toString(){
            switch (this){
                case GRAB:
                    return "GRAB";
                case OPEN:
                    return "OPEN";
                default:
                    return "noo";
            }
        };
    }

    public void switchClawState(){
        switch (clawState){
            case GRAB:
                clawState = ClawState.OPEN;
                setClawOpen();
                break;

            case OPEN:
                clawState = GRAB;
                setClawGrab();
                break;
        }
    }

    public void setClawOpen(){
        mClaw.setPosition(CLAW_OPEN);
        clawState = OPEN;
    }
    public void setClawGrab(){
        mClaw.setPosition(CLAW_GRAB);
        clawState = GRAB;
    }

    // Wrist
    public void setWristIntake_ParallelToGround(){
        mWrist.setPosition(WRIST_INTAKE_PARALLEL_GROUND);
    }
    public void setWristIntake(){
        mWrist.setPosition(WRIST_INTAKE);
    }
    public void setWristReleaseBox(){
        mWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
    }
    public void setWristOrigin(){
        mWrist.setPosition(WRIST_ORIGIN);
    }

    // Spin Wrist
    public void setSpinwristIntake(){
        mSpinWrist.setPosition(SPINWRIST_INTAKE);
    }
    public void setSpinwristReleaseclip(){
        mSpinWrist.setPosition(SPINWRIST_RELEASECLIP);
    }

    // Arm
    private int armTargetPosition;
    public void setArmPosition(int pos){
        armTargetPosition = pos;
        armPidCtrl.setTargetPosition(armTargetPosition);

    }
    public void resetArm(){
        if(armMag.isPressed()){
            mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //Slide
    public int slideTargetPosition;

    // TODO: AVOID TOO HIGH REVERSE POWER
    public void setSlidePosition(int pos){
        slideTargetPosition = pos;
        slideLeftPidCtrl_Horizontal.setTargetPosition(slideTargetPosition);
        slideLeftPidCtrl_Vertical.setTargetPosition(slideTargetPosition);

        slideRightPidCtrl_Horizontal.setTargetPosition(slideTargetPosition);
        slideRightPidCtrl_Vertical.setTargetPosition(slideTargetPosition);

        if(slideTargetPosition < getSlidePosition() && getSlidePosition() < 1000){
            setSlideOutputBounds(0.5);
        }else{
            setSlideOutputBounds(1);
        }
    }

    public void resetSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setPower(-0.2);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setPower(-0.2);

        delay(100);

        mSlideRight.setPower(0);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideLeft.setPower(0);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum SlideState{
        HORIZONTAL(slideLeftPidConf_Horizontal),
        VERTICAL(slideLeftPidConf_Vertical);

        private final PIDCoefficients pidCoefficients;
        SlideState(PIDCoefficients pidCoefficients) {
            this.pidCoefficients = pidCoefficients;
        }

        @Override
        public String toString(){
            switch (this){
                case HORIZONTAL:
                    return "HORIZONTAL";
                case VERTICAL:
                    return "VERTICAL";
                default:
                    return "noo";
            }
        };
    }
    public void switchSlideState(){
        switch (slideState){
            case HORIZONTAL:
                slideState = SlideState.VERTICAL;
                break;
            case VERTICAL:
                slideState = SlideState.HORIZONTAL;
                break;
        }
    }
    public void setSlideState(SlideState slideState){
        if(slideState == SlideState.HORIZONTAL){
            this.slideState = slideState;
        }
        if(slideState == SlideState.VERTICAL){
            this.slideState = slideState;
        }
    }
    public void setSlideOutputBounds(double bounds){
        for(PIDFController pidfController: slidePidCtrl){
            pidfController.setOutputBounds(-bounds,bounds);
        }
    }


    // Getter
    public int getArmLeftPosition(){
        return mArmLeft.getCurrentPosition();
    }
    public int getArmRightPosition(){
        return mArmRight.getCurrentPosition();
    }
    public int getArmPosition(){
        return  (getArmLeftPosition() + getArmRightPosition()) / 2;
    }

    public int getSlideLeftPosition(){
        return mSlideLeft.getCurrentPosition();
    }
    public int getSlideRightPosition(){
        return mSlideRight.getCurrentPosition();
    }
    public int getSlidePosition(){
        return (getSlideLeftPosition() + getSlideRightPosition()) / 2;
    }

    public double getArmLeftPower(){
        return mArmLeft.getPower();
    }
    public double getArmRightPower(){
        return mArmRight.getPower();
    }

    public int getArmTargetPosition(){
        return armTargetPosition;
    }
    public int getSlideTargetPosition(){
        return slideTargetPosition;
    }
    public double getSlideLeftPower(){
        return mSlideLeft.getPower();
    }
    public double getSlideRightPower(){
        return  mSlideRight.getPower();
    }


    protected void delay(int mm_time) {
        long end = System.currentTimeMillis() + mm_time;
        while (end > System.currentTimeMillis() && updateRunnable!=null) {
            updateRunnable.run();
        }
    }

}
