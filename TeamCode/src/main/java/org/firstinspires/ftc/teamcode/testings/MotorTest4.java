package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor test 4", group = "Testing")
public class MotorTest4 extends LinearOpMode {
    private DcMotorEx mLeftFront = null;
    private DcMotorEx mRightFront = null;
    private DcMotorEx mLeftBack = null;
    private DcMotorEx mRightBack = null;
    private double power = 1;
    @Override
    public void runOpMode(){
        mLeftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        mRightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        mLeftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        mRightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        waitForStart();


        while(opModeIsActive()){
            mRightFront.setPower(power);
            mLeftFront.setPower(power);
            mRightBack.setPower(power);
            mLeftBack.setPower(power);

            telemetry.addData("leftFront_power", mLeftFront.getPower());
            telemetry.addData("leftBack_power", mLeftBack.getPower());
            telemetry.addData("rightFront_power", mRightFront.getPower());
            telemetry.addData("rightBack_power", mLeftFront.getPower());

            telemetry.addData("leftFront_velocity",mLeftFront.getVelocity());
            telemetry.addData("leftBack_velocity",mLeftBack.getVelocity());
            telemetry.addData("rightFront_velocity",mRightFront.getVelocity());
            telemetry.addData("rightBack_velocity",mLeftFront.getVelocity());

            telemetry.update();
        }
    }
}
