package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Slide extends LinearOpMode {

    DcMotor slideMotor;
    double motorPower = 1.0;
    boolean lastButtonState = false; // 用于检测按键的变化
    boolean movingForward = true; // 用于切换伸展或收缩方向

    // 假设 1 米的滑动需要旋转的电机刻度数为 TICKS_PER_METER，根据实际情况设置
    static final int TICKS_PER_METER = 1000;

    @Override
    public void runOpMode() {
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.b;

            // 检测按钮的变化
            if (currentButtonState && !lastButtonState) {
                if (movingForward) {
                    slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + TICKS_PER_METER);
                } else {
                    slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - TICKS_PER_METER);
                }
                slideMotor.setPower(motorPower);
                movingForward = !movingForward;
            }
            lastButtonState = currentButtonState;
            sleep(50);
        }
    }
}