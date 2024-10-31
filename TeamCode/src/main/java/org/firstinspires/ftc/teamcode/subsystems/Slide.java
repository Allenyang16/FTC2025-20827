package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Slide extends LinearOpMode {

    DcMotor slideMotor;
    double motorPower = 1.0; // 电机功率的绝对值，1.0 表示全速
    boolean lastButtonState = false; // 用于检测按键的变化
    boolean movingForward = true; // 用于切换伸展或收缩方向

    // 假设 1 米的滑动需要旋转的电机刻度数为 TICKS_PER_METER
    static final int TICKS_PER_METER = 1000; // 根据实际情况设置

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化电机
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 等待比赛开始
        waitForStart();

        // 进入主控制循环
        while (opModeIsActive()) {
            boolean currentButtonState = gamepad1.b;

            // 检测按钮的变化
            if (currentButtonState && !lastButtonState) {
                // 如果按钮从未按下到按下，切换滑轨的移动方向
                if (movingForward) {
                    slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + TICKS_PER_METER);
                } else {
                    slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - TICKS_PER_METER);
                }
                slideMotor.setPower(motorPower); // 使电机以给定功率移动
                movingForward = !movingForward; // 每次按下时切换方向
            }

            // 更新按钮状态
            lastButtonState = currentButtonState;

            // 防止循环过快运行
            sleep(50);
        }
    }
}