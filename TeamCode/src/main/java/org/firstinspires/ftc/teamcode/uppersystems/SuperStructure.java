package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// TODO: 测所有的数和位置！！！！ 别不测数就跑
@Config
public class SuperStructure {
    private DcMotorEx mArm = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients slidePidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController slidePidCtrl;

    private Servo mMinorArm = null; // continuous
    private Servo mClaw = null;
    private Servo mSpinWrist = null;
    private Servo mWrist = null;

    public static int SLIDE_BOX_HIGH = 3100, SLIDE_BOX_LOW = 2200;
    public static int SLIDE_CHAMBER_HIGH = 1400, SLIDE_CHAMBER_LOW = 0;
    public static int SLIDE_INTAKE_MAX = 2000, SLIDE_MIN = 0;
    // TODO: 需要考虑一下把ARM的初始值设为什么
    public static int ARM_INTAKE = 1300;
    public static int ARM_RELEASE = 0;
    // WRIST
    public static double WRIST_ORIGIN = 0.28;
    public static double WRIST_INTAKE = 0.28;
    public static double WRIST_RELEASE_BOX_HIGH = 0.9, WRIST_RELEASE_BOX_LOW = 0.8;
    public static double WRIST_RELEASE_CHAMBER_HIGH = 0.9, WRIST_RELEASE_CHAMBER_LOW = 0.8;

    // Spin Wrist TODO: 刷固件
    public static double SPINWRIST_ORIGIN = 0;
    public static double SPINWRIST_INTAKE = 0.9;
    public static double SPINWRIST_RELEASE = 0;
    
    // Claw
    public static double CLAW_OPEN = 0;
    public static double CLAW_GRAB = 0.45;
    public ClawState clawState = ClawState.GRAB;

    private final LinearOpMode opMode;
    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure (LinearOpMode opMode){
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);
        slidePidCtrl = new PIDFController(slidePidConf);

        mArm = hardwareMap.get(DcMotorEx.class,"arm");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mMinorArm = hardwareMap.get(Servo.class,"miniArm");
        mClaw = hardwareMap.get(Servo.class,"claw");
        mWrist = hardwareMap.get(Servo.class,"wrist");
        mSpinWrist = hardwareMap.get(Servo.class,"spinWrist");

        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
    }

    // Claw
    public enum ClawState{
        OPEN(CLAW_OPEN),
        GRAB(CLAW_GRAB);
        private final double clawPosition;
        ClawState(double clawPosition) {
            this.clawPosition = clawPosition;
        }
    }

    public void switchClawState(){
        switch (clawState){
            case GRAB:
                clawState = ClawState.OPEN;
                mClaw.setPosition(CLAW_OPEN);

            case OPEN:
                clawState = ClawState.GRAB;
                mClaw.setPosition(CLAW_GRAB);
        }
    }

    // Wrist
    public void setWristIntake(){
        mWrist.setPosition(WRIST_INTAKE);
    }

    // Spin Wrist
    public void setSpinwristIntake(){
        mSpinWrist.setPosition(SPINWRIST_INTAKE);
    }

    // Small arm part
    public void setmArmSpin(double value){
        mMinorArm.setPosition(value);
    }
    // Arm
    private int armTargetPosition;

    // TODO: PID 测数等YBX回来再试
    public void setArmPosition(int pos){
        mArm.setTargetPosition(pos);
        mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(mArm.getCurrentPosition()<=400&&pos<=mArm.getCurrentPosition()){
            mArm.setPower(0.3);
        }else if(mArm.getCurrentPosition()<1300&&pos>=mArm.getCurrentPosition()){
            mArm.setPower(1);
        }else if(pos>=mArm.getCurrentPosition()){
            mArm.setPower(0.6);
        }else{
            mArm.setPower(1);
        }
    }
    // TODO: update change to null
    public void update() {

    }

    //Slide
    public int slideTargetPosition;
    // TODO: 先跑SlideTest, 将滑轨拉出来，试一下读数，如果有负的就需要Reverse
    public void setSlidePosition(int pos){
        mSlideLeft.setTargetPosition(pos);
        mSlideRight.setTargetPosition(pos);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mSlideRight.setPower(0.9);
        mSlideLeft.setPower(0.9);
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


}
