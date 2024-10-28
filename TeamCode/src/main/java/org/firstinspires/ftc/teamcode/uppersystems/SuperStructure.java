package org.firstinspires.ftc.teamcode.uppersystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SuperStructure {
    /*private DcMotorEx mArm = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    public static PIDCoefficients armPidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController armPidCtrl;

    public static PIDCoefficients slidePidConf = new PIDCoefficients(0.0025, 0.00011, 0.00013);
    private final PIDFController slidePidCtrl;

    private Servo mIntake = null; // continuous
    private Servo mClaw = null;
    private Servo mWristVertical = null;
    private Servo mWristHorizontal = null;

    public double CONTINUOUS_SPIN = 1, CONTINUOUS_STOP = 0.5, CONTINUOUS_SPIN_OPPOSITE = -0.5;
    public static int SLIDE_MAX = 573, SLIDE_MIN = 0;
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
        mIntake = hardwareMap.get(Servo.class,"intake");
        mClaw = hardwareMap.get(Servo.class,"claw");
        mWristHorizontal = hardwareMap.get(Servo.class,"wristHorizontal");
        mWristVertical = hardwareMap.get(Servo.class,"wristVertical");

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //mIntake.setDirection(Servo.Direction.REVERSE);
    }

    // Intake & Outtake part
    public void setIntakeSpin(double value){
        mIntake.setPosition(value);
    }
    public void intakeSpin(){
        setIntakeSpin(CONTINUOUS_SPIN);
    }
    public void intakeStop(){
        setIntakeSpin(CONTINUOUS_STOP);
    }
    public void intakeSpinOpposite(){
        setIntakeSpin(CONTINUOUS_SPIN_OPPOSITE);
    }

    // Arm
    private int armTargetPosition;
    public void setArmPosition(int pos){
        armTargetPosition = pos;
        mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPidCtrl.setOutputBounds(-0.9,0.9);
    }
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
        setArmPosition(ARM_INTAKE_FAR);
        mWristVertical.setPosition(WRIST_INTAKE_FAR);
        setSlidePosition(SLIDE_MAX);
    }
    public void intakeNear(){
        setArmPosition(ARM_INTAKE_LOW);
        mWristVertical.setPosition(WRIST_INTAKE_NEAR);
        setSlidePosition(SLIDE_MIN);
    }

    // Release Action
    // Chamber
    public void chamberHigh() {
        setArmPosition(ARM_RELEASE_CHAMBER_HIGH);
        mWristVertical.setPosition(WRIST_RELEASE_CHAMBER_HIGH);
        setSlidePosition(SLIDE_MAX);
    }
    public void chamberLow() {
        setArmPosition(ARM_RELEASE_CHAMBER_LOW);
        mWristVertical.setPosition(WRIST_RELEASE_CHAMBER_LOW);
        setSlidePosition(SLIDE_MIN);
    }
    //Box
    public void boxHigh() {
        setArmPosition(ARM_RELEASE_BOX_HIGH);
        mWristVertical.setPosition(WRIST_RELEASE_BOX_HIGH);
        setSlidePosition(SLIDE_MAX);
    }
    public void boxLow() {
        setArmPosition(ARM_RELEASE_BOX_LOW);
        mWristVertical.setPosition(WRIST_RELEASE_BOX_LOW);
        setSlidePosition(SLIDE_MIN);
    }*/

}
