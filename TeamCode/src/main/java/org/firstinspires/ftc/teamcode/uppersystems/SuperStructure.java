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

    public static PIDCoefficients armLeftPidConf = new PIDCoefficients(0.009, 0.00015, 0.00033);
    private final PIDFController armLeftPidCtrl;

    public static PIDCoefficients armRightPidConf = new PIDCoefficients(0.009, 0.00015, 0.00033);
    private final PIDFController armRightPidCtrl;

    public static PIDCoefficients slideLeftPidConf_Horizontal = new PIDCoefficients(0.004, 0.00007, 0.00);
    private final PIDFController slideLeftPidCtrl_Horizontal;
    public static PIDCoefficients slideLeftPidConf_Vertical = new PIDCoefficients(0.0095, 0.00007, 0.00);
    private final PIDFController slideLeftPidCtrl_Vertical;

    public static PIDCoefficients slideRightPidConf_Horizontal = new PIDCoefficients(0.004, 0.00007, 0.00);
    private final PIDFController slideRightPidCtrl_Horizontal;
    public static PIDCoefficients slideRightPidConf_Vertical = new PIDCoefficients(0.009, 0.00007, 0.00);
    private final PIDFController slideRightPidCtrl_Vertical;

    public static PIDCoefficients slideLeftPidConf_Hang = new PIDCoefficients(0.004, 0.003, 0.00);
    private final PIDFController slideLeftPidCtrl_Hang;
    public static PIDCoefficients slideRightPidConf_Hang = new PIDCoefficients(0.009, 0.003, 0.00);
    private final PIDFController slideRightPidCtrl_Hang;

    private List<PIDFController> slidePidCtrl;

    private Servo mClaw = null;
    private Servo mSpinWrist = null;
    private Servo mWrist = null;

    private TouchSensor armMag = null;
    private SpinWrist wristMode = SpinWrist.DEG_45;

    public static int SLIDE_BOX_HIGH = 1750, SLIDE_BOX_LOW = 500;
    public static int SLIDE_CHAMBER_HIGH = 780, SLIDE_CHAMBER_HIGH_AUTO = 500, SLIDE_CHAMBER_LOW = 0;
    public static int SLIDE_CHAMBER_HIGH_DOWN = 420;
    public static int SLIDE_CHAMBER_HIGH_TELEOP = 540;
    public static int SLIDE_CHAMBER_HIGH_DOWN_TELEOP = 360;
    public static int SLIDE_INTAKE_MAX = 700, SLIDE_AUTO = 70, SLIDE_MIN = 0;
    public static int SLIDE_HANG_AUTO = 200, SLIDE_HANG_HIGH_UP = 1300, SLIDE_HANG_HIGH_DOWN = 600;
    public static int SLIDE_HANG_LOW_UP = 1200, SLIDE_HANG_LOW_DOWN = 0;

    public static int ARM_CHAMBER_HIGH_Test = 220;
    public static int ARM_INTAKE = -970;
    public static int ARM_PRE_INTAKE = -890;
    public static int ARM_POST_INTAKE = -890;
    // TODO: CHECK THIS VALUE
    public static int ARM_INTAKE_SPECIMEN = 700;
    public static int ARM_RELEASE_BOX = 95;
    public static int ARM_RELEASE_CHAMBER = -320, ARM_RELEASE_CHAMBER_TELEOP = 340;// 80 for teleOp

    public static int ARM_HANG_HIGH = -200, ARM_HANG_AUTO = -220;//
    public static int ARM_HANG_LOW = 350;

    public double slidePower = 0;
    public double armPower = 0;

    //New Auto
    public static int ARM_AUTO_CHAMBER = 160;


    // WRIST
    public static double WRIST_INTAKE = 0.17, WRIST_INTAKE_PARALLEL_GROUND = 0.75;
    public static double WRIST_INTAKE_SPECIMEN = 0.9, WRIST_INTAKE_SPECIMEN_GROUND = 0.4
            ;

    // TODO: Retest
    public static double WRIST_RELEASE_BOX_HIGH = 0.85, WRIST_RELEASE_BOX_LOW = 0.28;
    public static double WRIST_RELEASE_CHAMBER_HIGH = 0.75, WRIST_RELEASE_CHAMBER_LOW = 0.8;
    public static double WRIST_RELEASE_CHAMBER_TELEOP = 0.75;
    // Spin Wrist
    public static double SPINWRIST_INTAKE = 0.97;

    public static double SPINWRIST_INTAKE_0 = 0.12;
    public static double SPINWRIST_INTAKE_45 = 0.3;
    public static double SPINWRIST_INTAKE_90 = 0.48;

    // TODO: CHANGE THE VALUE
    public static double SPINWRIST_INTAKE_CLOCKWISE = 0.75;
    public static double SPINWRIST_INTAKE_COUNTERCLOCKWISE = 0.75;
    public static double SPINWRIST_INTAKE_SPECIMEN = 0.86;
    public static double SPINWRIST_RELEASE_SPECIMEN = 0.12;
    public static double SPINWRIST_AUTO = 0.77;
    
    // Claw
    // TODO: TEST Value
    public static double CLAW_OPEN = 0.25;
    public static double CLAW_OPENLarge = 0.33;
    public static double CLAW_GRAB = 0.05;
    public ClawState clawState = GRAB;
    public SlideState slideState = SlideState.VERTICAL;
    public WristIntakeState wristIntakeState = WristIntakeState.PRE_INTAKE;
    public static boolean PID_isUsed = true;
    private final LinearOpMode opMode;
    private Runnable updateRunnable;

    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }

    public SuperStructure (LinearOpMode opMode){
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        armLeftPidCtrl = new PIDFController(armLeftPidConf);
        armRightPidCtrl = new PIDFController(armRightPidConf);

        slideLeftPidCtrl_Horizontal = new PIDFController(slideLeftPidConf_Horizontal);
        slideLeftPidCtrl_Vertical = new PIDFController(slideLeftPidConf_Vertical);

        slideRightPidCtrl_Horizontal = new PIDFController(slideRightPidConf_Horizontal);
        slideRightPidCtrl_Vertical = new PIDFController(slideRightPidConf_Vertical);

        slideRightPidCtrl_Hang = new PIDFController(slideRightPidConf_Hang);
        slideLeftPidCtrl_Hang = new PIDFController(slideLeftPidConf_Hang);

        slidePidCtrl = Arrays.asList(slideLeftPidCtrl_Horizontal,slideLeftPidCtrl_Vertical,slideRightPidCtrl_Horizontal,slideRightPidCtrl_Vertical,slideRightPidCtrl_Hang,slideLeftPidCtrl_Hang);

        mArmLeft = hardwareMap.get(DcMotorEx.class,"armLeft");
        mArmRight = hardwareMap.get(DcMotorEx.class, "armRight");

        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        mClaw = hardwareMap.get(Servo.class,"claw");
        mWrist = hardwareMap.get(Servo.class,"wrist");
        mSpinWrist = hardwareMap.get(Servo.class,"spinWrist");

//        armMag = hardwareMap.get(TouchSensor.class,"armMag");
        mSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mArmRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mArmLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        mArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // init
    public void initialize(){
        setSpinWristIntake_specimen();
        setWristPostRelease();
        setClawGrab();
    }
    public void reset(){
        mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //update
    public void update() {
        mArmLeft.setPower(armLeftPidCtrl.update(getArmLeftPosition()));
        mArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mArmRight.setPower(armRightPidCtrl.update(getArmRightPosition()));
        mArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(slideState == SlideState.HORIZONTAL){
            mSlideRight.setPower(slideRightPidCtrl_Horizontal.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slideLeftPidCtrl_Horizontal.update(mSlideLeft.getCurrentPosition()));
        } else if (slideState == SlideState.VERTICAL) {
            mSlideRight.setPower(slideRightPidCtrl_Vertical.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slideLeftPidCtrl_Vertical.update(mSlideLeft.getCurrentPosition()));
        } else if (slideState == SlideState.HANG){
            mSlideRight.setPower(slideRightPidCtrl_Hang.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slideLeftPidCtrl_Hang.update(mSlideLeft.getCurrentPosition()));
        } else {
            mSlideRight.setPower(slideRightPidCtrl_Vertical.update(mSlideRight.getCurrentPosition()));
            mSlideLeft.setPower(slideLeftPidCtrl_Vertical.update(mSlideLeft.getCurrentPosition()));
        }
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        packet.put("Slide State", slideState.toString());
        packet.put("Wrist state", wristIntakeState.toString());
        packet.put("rightSlide_encoder: ", getSlideRightPosition());
        packet.put("leftSlide_encoder: ", getSlideLeftPosition());
        packet.put("leftSlide_power:", getSlideLeftPower());
        packet.put("rightSlide_power", getSlideRightPower());
        packet.put("slideTargetPos:", getSlideTargetPosition());

        packet.put("rightArm_pos: ", getArmRightPosition());
        packet.put("leftArm_pos: ",getArmLeftPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // Hang
    public void hang_setSlide(int pos){
        setSlidePosition_hang(pos);
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
        delay(200);
        clawState = OPEN;
    }
    public void setClawOpenLarge(){
        mClaw.setPosition(CLAW_OPENLarge);
        delay(200);
        clawState = OPEN;
    }

    public void setClawGrab(){
        mClaw.setPosition(CLAW_GRAB);
        delay(200);
        clawState = GRAB;
    }

    // Wrist

    // TODO: Pre Intake Position
    public void setWristPreIntake(){
        mWrist.setPosition(WRIST_INTAKE_PARALLEL_GROUND);
        wristIntakeState = WristIntakeState.PRE_INTAKE;
    }
    public void setWristIntake(){
        mWrist.setPosition(WRIST_INTAKE);
        wristIntakeState = WristIntakeState.INTAKE;
    }

    public void setWristIntakeSpecimen(){
        mWrist.setPosition(WRIST_INTAKE_SPECIMEN);
        wristIntakeState = WristIntakeState.INTAKE_SPECIMEN;
    }
    public void setWristIntakeSpecimenGround(){
        mWrist.setPosition(WRIST_INTAKE_SPECIMEN_GROUND);
        wristIntakeState = WristIntakeState.INTAKE_SPECIMEN;
    }

    public void setWristPostRelease(){
        mWrist.setPosition(WRIST_INTAKE);
        wristIntakeState = WristIntakeState.PRE_INTAKE;
    }
    public void setWristReleaseBox(){
        mWrist.setPosition(WRIST_RELEASE_BOX_HIGH);
        wristIntakeState = WristIntakeState.RELEASE_SAMPLE;
    }
    public void setWristReleaseChamber(){
        mWrist.setPosition(WRIST_INTAKE_PARALLEL_GROUND);
        wristIntakeState = WristIntakeState.RELEASE_SPECIMEN;
    }
    public void setWristReleaseChamberTeleop(){
        mWrist.setPosition(WRIST_RELEASE_CHAMBER_TELEOP);
        wristIntakeState = WristIntakeState.RELEASE_SPECIMEN;
    }

    public enum WristIntakeState {
        INTAKE(WRIST_INTAKE),
        PRE_INTAKE(WRIST_INTAKE_PARALLEL_GROUND),
        INTAKE_SPECIMEN(WRIST_INTAKE_SPECIMEN_GROUND),
        RELEASE_SAMPLE(WRIST_RELEASE_BOX_HIGH),
        RELEASE_SPECIMEN(WRIST_RELEASE_CHAMBER_HIGH);

        private final double wristPosition;
        WristIntakeState(double wristPosition) {
            this.wristPosition = wristPosition;
        }
        @Override
        public String toString(){
            switch (this){
                case INTAKE:
                    return "INTAKE";
                case PRE_INTAKE:
                    return "PRE_INTAKE";
                case INTAKE_SPECIMEN:
                    return "INTAKE_SPECIMEN";
                case RELEASE_SAMPLE:
                    return "RELEASE_SAMPLE";
                default:
                    return "none";
            }
        };
    }

    public void switchWristIntakeState(){
        switch (wristIntakeState){
            case INTAKE:
                wristIntakeState = WristIntakeState.PRE_INTAKE;
                setWristPreIntake();
                break;
            case PRE_INTAKE:
                wristIntakeState = WristIntakeState.INTAKE;
                setWristIntake();
                break;
        }
    }
    public double translation_coefficient(){
        if(wristIntakeState == WristIntakeState.INTAKE){
            return 0.4;
        } else if (wristIntakeState == WristIntakeState.PRE_INTAKE) {
            return 1.0;
        } else if (wristIntakeState == WristIntakeState.INTAKE_SPECIMEN) {
            return 0.3;
        } else if (wristIntakeState == WristIntakeState.RELEASE_SAMPLE) {
            if(getSlidePosition() < 1500){
                return 0.9;
            }else{
                return 0.5;
            }
        } else if (wristIntakeState == WristIntakeState.RELEASE_SPECIMEN) {
            return 1.0;
        } else{
            return 1.0;
        }
    }

    public double heading_coefficient(){
        if(wristIntakeState == WristIntakeState.INTAKE){
            return 0.35;
        } else if (wristIntakeState == WristIntakeState.PRE_INTAKE) {
            return 1.0;
        } else if (wristIntakeState == WristIntakeState.INTAKE_SPECIMEN) {
            return 0.2;
        } else if (wristIntakeState == WristIntakeState.RELEASE_SAMPLE) {
            if(getSlidePosition() < 2500){
                return 0.5;
            }else{
                return 0.3;
            }
        } else if (wristIntakeState == WristIntakeState.RELEASE_SPECIMEN) {
            return 0.6;
        } else{
            return 1.0;
        }
    }


    // Spin Wrist
    public void setSpinWristIntake(){
        mSpinWrist.setPosition(SPINWRIST_INTAKE);
    }
    public void setSpinWristReleaseBox(){
        mSpinWrist.setPosition(SPINWRIST_INTAKE_SPECIMEN);
    }
    public void setSpinWristIntake_specimen(){
        mSpinWrist.setPosition(SPINWRIST_INTAKE_SPECIMEN);
    }
    public void setSpinWristRelease_specimen(){
        mSpinWrist.setPosition(SPINWRIST_RELEASE_SPECIMEN);
    }
    public void setSpinWristIntake_spinClockwise(){
        wristMode = SpinWrist.DEG_0;
        mSpinWrist.setPosition(wristMode.spinWristVal);
    }

    public void setSpinWristIntake_spinCounterClockwise(){
        switch (wristMode) {
            case DEG_0:
                wristMode = SpinWrist.DEG_45;
                break;
            case DEG_45:
                wristMode = SpinWrist.DEG_90;
                break;
        }
        mSpinWrist.setPosition(wristMode.spinWristVal);
    }

    public void setSpinWristAuto(){
        mSpinWrist.setPosition(SPINWRIST_AUTO);
    }


    public enum SpinWrist {
        DEG_0(SPINWRIST_INTAKE_0),
        DEG_45(SPINWRIST_INTAKE_45),
        DEG_90(SPINWRIST_INTAKE_90);

        private final double spinWristVal;
        SpinWrist(double spinWristVal) {
            this.spinWristVal = spinWristVal;
        }
    }


    // Arm
    private int armTargetPosition;
    public void setArmPosition(int pos){
        wristMode = SpinWrist.DEG_0;
        armTargetPosition = pos;
        armLeftPidCtrl.setTargetPosition(armTargetPosition);
        armRightPidCtrl.setTargetPosition(armTargetPosition);
        if(armTargetPosition > getArmPosition() && getArmPosition() > 700){
            armRightPidCtrl.setOutputBounds(-0.5,0.5);
            armLeftPidCtrl.setOutputBounds(-0.5,0.5);
        }else{
            armRightPidCtrl.setOutputBounds(-1,1);
            armLeftPidCtrl.setOutputBounds(-1,1);
        }
    }
    public void disableArmMotor(){
        mArmLeft.setMotorDisable();
        mArmRight.setMotorDisable();
    }
    public void enableArmMotor(){
        mArmLeft.setMotorEnable();
        mArmRight.setMotorEnable();
    }


    public void resetArm(){
        mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        if(armMag.isPressed()){
//            mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
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

        if(slideState == SlideState.VERTICAL){
            if(slideTargetPosition < getSlidePosition() && getSlidePosition() < 200){
                setSlideOutputBounds(0.2);
            } else if (slideTargetPosition < getSlidePosition() && getSlidePosition() < 600) {
                setSlideOutputBounds(0.9);
            } else{
                setSlideOutputBounds(1);
            }
        }else{
            setSlideOutputBounds(1);
        }

    }
    public void setSlidePosition_horizontal(int pos){
        slideState = SlideState.HORIZONTAL;
        delay(50);
        slideTargetPosition = pos;
        slideLeftPidCtrl_Horizontal.setTargetPosition(slideTargetPosition);
        slideRightPidCtrl_Horizontal.setTargetPosition(slideTargetPosition);
    }

    public void setSlidePosition_verticle(int pos){
        slideState = SlideState.VERTICAL;
        delay(50);
        slideTargetPosition = pos;
        slideLeftPidCtrl_Vertical.setTargetPosition(slideTargetPosition);
        slideRightPidCtrl_Vertical.setTargetPosition(slideTargetPosition);
        if(slideTargetPosition < getSlidePosition() && getSlidePosition() < 200){
            setSlideOutputBounds(0.2);
        } else if (slideTargetPosition < getSlidePosition() && getSlidePosition() < 600) {
            setSlideOutputBounds(0.9);
        } else {
            setSlideOutputBounds(1);
        }
    }

    public void setSlidePosition_hang(int pos){
        slideState = SlideState.HANG;
        delay(50);
        slideTargetPosition = pos;
        slideLeftPidCtrl_Hang.setTargetPosition(slideTargetPosition);
        slideRightPidCtrl_Hang.setTargetPosition(slideTargetPosition);
    }
    public void setSlidePower(double power){
        mSlideRight.setPower(power);
        mSlideLeft.setPower(power);
    }

    public void resetSlide(){
        mSlideRight.setPower(0);
        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSlideLeft.setPower(0);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum SlideState{
        HORIZONTAL(slideLeftPidConf_Horizontal),
        VERTICAL(slideLeftPidConf_Vertical),
        HANG(slideLeftPidConf_Hang);


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
                case HANG:
                    return "HANG";
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

    public void setUpperHang(){
        mSlideLeft.setPower(slidePower);
        mSlideRight.setPower(slidePower);
        mArmLeft.setPower(armPower);
        mArmRight.setPower(armPower);

        mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
