package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ToggleClawControl", group="Robot")
public class Claw extends LinearOpMode {

    private Servo clawServo;

    private static final double CLAW_OPEN_POSITION = 0.0;    // 完全打开位置
    private static final double CLAW_CLOSED_POSITION = 0.45;  // 完全关闭位置

    private boolean clawClosed = false;  // 初始状态设为打开
    private boolean lastButtonState = false;  // 记录上一次按钮的状态

    @Override
    public void runOpMode() {
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // 设置爪子的初始位置为打开
        clawServo.setPosition(CLAW_OPEN_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // 检测按键是否被按下，并判断是否从未按下状态变为按下状态
            boolean currentButtonState = gamepad1.a;
            if (currentButtonState && !lastButtonState) {
                // 切换爪子状态
                clawClosed = !clawClosed;

                // 根据新状态调整伺服位置
                if (clawClosed) {
                    clawServo.setPosition(CLAW_CLOSED_POSITION);
                } else {
                    clawServo.setPosition(CLAW_OPEN_POSITION);
                }
            }

            // 更新按钮状态
            lastButtonState = currentButtonState;

            // 输出调试信息
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.addData("Claw Closed", clawClosed);
            telemetry.update();
        }
    }
}
