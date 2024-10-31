package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private Servo mLeftWrist = null;
    private Servo mRightWrist = null;

    public static int SLIDE_MAX = 573, SLIDE_MIN = 0;
    // TODO: 需要考虑一下把ARM的初始值设为什么
    public static int ARM_INTAKE_FAR = 600, ARM_INTAKE_LOW = 0;
    public static int ARM_RELEASE_BOX_HIGH = 3000, ARM_RELEASE_BOX_LOW = 2000;
    public static int ARM_RELEASE_CHAMBER_HIGH = 1000, ARM_RELEASE_CHAMBER_LOW = 800;
    // WRIST
    public static double WRIST_ORIGIN = 0;
    public static double WRIST_INTAKE_FAR = 0.7, WRIST_INTAKE_NEAR = 0.5;
    public static double WRIST_RELEASE_BOX_HIGH = 0.9, WRIST_RELEASE_BOX_LOW = 0.8;
    public static double WRIST_RELEASE_CHAMBER_HIGH = 0.9, WRIST_RELEASE_CHAMBER_LOW = 0.8;

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

        mArm = hardwareMap.get(DcMotorEx.class,"Arm");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mMinorArm = hardwareMap.get(Servo.class,"miniArm");
        mClaw = hardwareMap.get(Servo.class,"claw");
        mRightWrist = hardwareMap.get(Servo.class,"rightWrist");
        mLeftWrist = hardwareMap.get(Servo.class,"leftWrist");

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
        setArmPosition(ARM_INTAKE_FAR);
        mLeftWrist.setPosition(WRIST_INTAKE_FAR);
        mRightWrist.setPosition(WRIST_INTAKE_FAR);
        setSlidePosition(SLIDE_MAX);
    }
    public void intakeNear(){
        setArmPosition(ARM_INTAKE_LOW);
        mLeftWrist.setPosition(WRIST_INTAKE_NEAR);
        mRightWrist.setPosition(WRIST_INTAKE_NEAR);
        setSlidePosition(SLIDE_MIN);
    }

    // Release Action
    // Chamber
    public void chamberHigh() {
        setArmPosition(ARM_RELEASE_CHAMBER_HIGH);
        mLeftWrist.setPosition(WRIST_RELEASE_CHAMBER_HIGH);
        mRightWrist.setPosition(WRIST_RELEASE_CHAMBER_HIGH);
        setSlidePosition(SLIDE_MAX);
    }
    public void chamberLow() {
        setArmPosition(ARM_RELEASE_CHAMBER_LOW);
        mLeftWrist.setPosition(WRIST_RELEASE_CHAMBER_LOW);
        mRightWrist.setPosition(WRIST_RELEASE_CHAMBER_LOW);
        setSlidePosition(SLIDE_MIN);
    }
    //Box
    public void boxHigh() {
        setArmPosition(ARM_RELEASE_BOX_HIGH);
        mLeftWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
        mRightWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
        setSlidePosition(SLIDE_MAX);
    }
    public void boxLow() {
        setArmPosition(ARM_RELEASE_BOX_LOW);
        mLeftWrist.setPosition(WRIST_RELEASE_BOX_LOW);
        mRightWrist.setPosition(WRIST_RELEASE_BOX_LOW);
        setSlidePosition(SLIDE_MIN);
    }

}
