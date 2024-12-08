package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Reset extends LinearOpMode {
    private DcMotorEx mArmLeft = null;
    private DcMotorEx mArmRight = null;
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;

    @Override public void runOpMode() {
        mArmLeft = hardwareMap.get(DcMotorEx.class,"armLeft");
        mArmRight = hardwareMap.get(DcMotorEx.class, "armRight");

        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
        }
    }
}
