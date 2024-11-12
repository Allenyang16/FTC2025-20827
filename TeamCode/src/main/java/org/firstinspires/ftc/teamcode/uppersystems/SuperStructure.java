package org.firstinspires.ftc.teamcode.uppersystems;

import static org.firstinspires.ftc.teamcode.uppersystems.SuperStructure.ClawState.GRAB;

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

@Config
public class SuperStructure {
    private TelemetryPacket packet = new TelemetryPacket();

    private DcMotorEx mArm = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients slidePidConf_Horizontal = new PIDCoefficients(0.0026, 0.00, 0.00);
    private final PIDFController slidePidCtrl_Horizontal;
    public static PIDCoefficients slidePidConf_Vertical = new PIDCoefficients(0.0026, 0.00, 0.00);
    private final PIDFController slidePidCtrl_Vertical;

    private Servo mMinorArm = null; // continuous
    private Servo mClaw = null;
    private Servo mSpinWrist = null;
    private Servo mWrist = null;
    private Servo mCalibrator = null;

    public static int SLIDE_BOX_HIGH = 3200, SLIDE_BOX_LOW = 1500;
    public static int SLIDE_CHAMBER_HIGH = 1400, SLIDE_CHAMBER_LOW = 0;
    public static int SLIDE_INTAKE_MAX = 1500, SLIDE_MIN = 0;
    // TODO: 需要考虑一下把ARM的初始值设为什么
    public static int ARM_INTAKE = 1300;
    public static int ARM_POST_INTAKE = 1000;
    public static int ARM_RELEASE = 0;
    // WRIST
    public static double WRIST_ORIGIN = 0.17;
    public static double WRIST_INTAKE = 0.17,WRIST_INTAKE_PARALLEL_GROUND = 0.4;
    public static double WRIST_RELEASE_BOX_HIGH = 0.55, WRIST_RELEASE_BOX_LOW = 0.28;
    public static double WRIST_RELEASE_CHAMBER_HIGH = 0.28, WRIST_RELEASE_CHAMBER_LOW = 0.8;

    // Spin Wrist
    public static double SPINWRIST_DEFAULT = 0;
    public static double SPINWRIST_RELEASECLIP = 0.81;
    
    // Claw
    public static double CLAW_OPEN = 0;
    public static double CLAW_GRAB = 0.33;
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
        slidePidCtrl_Horizontal = new PIDFController(slidePidConf_Horizontal);
        slidePidCtrl_Vertical = new PIDFController(slidePidConf_Vertical);

        mArm = hardwareMap.get(DcMotorEx.class,"arm");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mClaw = hardwareMap.get(Servo.class,"claw");
        mWrist = hardwareMap.get(Servo.class,"wrist");
        mSpinWrist = hardwareMap.get(Servo.class,"spinWrist");
        mCalibrator = hardwareMap.get(Servo.class,"ruler");

        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mArm.setDirection(DcMotorSimple.Direction.REVERSE);

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // init
    public void initialize(){
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setSpinwristReleaseclip();
        setWristIntake();
        setClawGrab();
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
    }
    public void setClawGrab(){
        mClaw.setPosition(CLAW_GRAB);
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

    // Spin Wrist
    public void setSpinwristIntake(){
        mSpinWrist.setPosition(SPINWRIST_DEFAULT);
    }
    public void setSpinwristReleaseclip(){
        mSpinWrist.setPosition(SPINWRIST_RELEASECLIP);
    }

    // Small arm part
    public void setmArmSpin(double value){
        mMinorArm.setPosition(value);
    }
    // Arm
    private int armTargetPosition;

    // TODO: PID 测数等YBX回来再试
    public void setArmPosition(int pos){
//        mArm.setTargetPosition(pos);
//        mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(mArm.getCurrentPosition()<=400&&pos<=mArm.getCurrentPosition()){
//            mArm.setPower(0.3);
//        }else if(mArm.getCurrentPosition()<1300&&pos>=mArm.getCurrentPosition()){
//            mArm.setPower(1);
//        }else if(pos>=mArm.getCurrentPosition()){
//            mArm.setPower(0.6);
//        }else{
//            mArm.setPower(1);
//        }
        armTargetPosition = pos;
        armPidCtrl.setTargetPosition(armTargetPosition);

    }
    // TODO: update change to null
    public void update() {
        mArm.setPower(armPidCtrl.update(mArm.getCurrentPosition()));
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(slideState == SlideState.HORIZONTAL){
            mSlideRight.setPower(slidePidCtrl_Horizontal.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slidePidCtrl_Horizontal.update(mSlideLeft.getCurrentPosition()));
        } else if (slideState == SlideState.VERTICAL) {
            mSlideRight.setPower(slidePidCtrl_Vertical.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slidePidCtrl_Vertical.update(mSlideLeft.getCurrentPosition()));
        }
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        packet.put("Claw State", clawState.toString());
        packet.put("Slide State", slideState.toString());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    //Slide
    public int slideTargetPosition;

    public void setSlidePosition(int pos){
        slideTargetPosition = pos;
        slidePidCtrl_Horizontal.setTargetPosition(slideTargetPosition);
        slidePidCtrl_Vertical.setTargetPosition(slideTargetPosition);
    }

    public void resetSlide(){
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setPower(-0.3);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setPower(-0.3);

        delay(50);

        mSlideRight.setPower(0);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideLeft.setPower(0);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum SlideState{
        HORIZONTAL(slidePidConf_Horizontal),
        VERTICAL(slidePidConf_Vertical);

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

    //Intake Action
    public void intakeFar(){
        setArmPosition(ARM_INTAKE);
        mSpinWrist.setPosition(WRIST_INTAKE);
        mWrist.setPosition(WRIST_INTAKE);
        setSlidePosition(SLIDE_INTAKE_MAX);
    }
    public void intakeNear(){
        setArmPosition(ARM_INTAKE);
        mSpinWrist.setPosition(WRIST_INTAKE);
        mWrist.setPosition(WRIST_INTAKE);
        setSlidePosition(SLIDE_MIN);
    }

    // Release Action
    // Chamber
    public void chamberHigh() {
        setArmPosition(ARM_RELEASE);
        mSpinWrist.setPosition(WRIST_RELEASE_CHAMBER_HIGH);
        mWrist.setPosition(WRIST_RELEASE_CHAMBER_HIGH);
        setSlidePosition(SLIDE_CHAMBER_HIGH);
    }
    public void chamberLow() {
        setArmPosition(ARM_RELEASE);
        mSpinWrist.setPosition(WRIST_RELEASE_CHAMBER_LOW);
        mWrist.setPosition(WRIST_RELEASE_CHAMBER_LOW);
        setSlidePosition(SLIDE_CHAMBER_LOW);
    }
    //Box
    public void boxHigh() {
        setArmPosition(ARM_RELEASE);
        mSpinWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
        mWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
        setSlidePosition(SLIDE_BOX_HIGH);
    }
    public void boxLow() {
        setArmPosition(ARM_RELEASE);
        mSpinWrist.setPosition(WRIST_RELEASE_BOX_LOW);
        mWrist.setPosition(WRIST_RELEASE_BOX_LOW);
        setSlidePosition(SLIDE_BOX_LOW);
    }


    // Getter
    public int getArmPosition(){
        return mArm.getCurrentPosition();
    }
    public int getSlideLeftPosition(){
        return mSlideLeft.getCurrentPosition();
    }
    public int getSlideRightPosition(){
        return mSlideRight.getCurrentPosition();
    }
    public int getSlidePosition(){
        return (getSlideLeftPosition()+getSlideRightPosition())/2;
    }
    public double getArmPower(){
        return mArm.getPower();
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
