package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SuperStructure_PID {
    private DcMotorEx mArm = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients slidePidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController slidePidCtrl;

    private Servo mMinorArm = null;
    private Servo mClaw = null;
    private Servo mLeftWrist = null;
    private Servo mRightWrist = null;

    public static int SLIDE_BOX_MAX = 3100, SLIDE_BOX_MIN = 2200;
    public static int SLIDE_CHAMBER_MAX = 1400, SLIDE_CHAMBER_MIN = 0;
    public static int SLIDE_INTAKE_MAX = 2000, SLIDE_INTAKE_MIN = 0;
    // TODO: 需要考虑一下把ARM的初始值设为什么
    public static int ARM_INTAKE = 1400;
    public static int ARM_RELEASE = 0;
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

    public SuperStructure_PID(LinearOpMode opMode){
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armPidCtrl = new PIDFController(armPidConf);
        slidePidCtrl = new PIDFController(slidePidConf);

        mArm = hardwareMap.get(DcMotorEx.class,"Arm");
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mMinorArm = hardwareMap.get(Servo.class,"miniArm");
        mClaw = hardwareMap.get(Servo.class,"claw");
        mLeftWrist = hardwareMap.get(Servo.class,"leftWrist");
        mRightWrist = hardwareMap.get(Servo.class,"rightWrist");

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
        armTargetPosition = pos;
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPidCtrl.setOutputBounds(-0.9,0.9);
    }
    // TODO: update change to null
    public void update() {
        mArm.setPower(armPidCtrl.update(mArm.getCurrentPosition() - armTargetPosition));
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideRight.setPower(slidePidCtrl.update(mSlideRight.getCurrentPosition()-slideTargetPosition));
        mSlideLeft.setPower(slidePidCtrl.update(mSlideLeft.getCurrentPosition()-slideTargetPosition));
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Slide
    public int slideTargetPosition;
    public void setSlidePosition(int pos){
        slideTargetPosition = pos;
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidePidCtrl.setOutputBounds(-1,1);
    }

    //Intake Action
    public void intakeFar(){
        setArmPosition(ARM_INTAKE);
        mLeftWrist.setPosition(WRIST_INTAKE_FAR);
        mRightWrist.setPosition(WRIST_INTAKE_FAR);
        setSlidePosition(SLIDE_INTAKE_MAX);
    }
    public void intakeNear(){
        setArmPosition(ARM_INTAKE);
        mLeftWrist.setPosition(WRIST_INTAKE_NEAR);
        mRightWrist.setPosition(WRIST_INTAKE_NEAR);
        setSlidePosition(SLIDE_INTAKE_MIN);
    }

    // Release Action
    // Chamber
    public void chamberHigh() {
        setArmPosition(ARM_RELEASE);
        mLeftWrist.setPosition(WRIST_RELEASE_CHAMBER_HIGH);
        mRightWrist.setPosition(WRIST_RELEASE_CHAMBER_HIGH);
        setSlidePosition(SLIDE_CHAMBER_MAX);
    }
    public void chamberLow() {
        setArmPosition(ARM_RELEASE);
        mLeftWrist.setPosition(WRIST_RELEASE_CHAMBER_LOW);
        mRightWrist.setPosition(WRIST_RELEASE_CHAMBER_LOW);
        setSlidePosition(SLIDE_CHAMBER_MIN);
    }
    //Box
    public void boxHigh() {
        setArmPosition(ARM_RELEASE);
        mLeftWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
        mRightWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
        setSlidePosition(SLIDE_BOX_MAX);
    }
    public void boxLow() {
        setArmPosition(ARM_RELEASE);
        mLeftWrist.setPosition(WRIST_RELEASE_BOX_LOW);
        mRightWrist.setPosition(WRIST_RELEASE_BOX_LOW);
        setSlidePosition(SLIDE_BOX_MIN);
    }

}
