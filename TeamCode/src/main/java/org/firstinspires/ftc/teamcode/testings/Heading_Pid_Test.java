package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

@TeleOp
@Config
public class Heading_Pid_Test extends LinearOpMode {
    // 可在FTC Dashboard上调整的PID参数
    public static double kP_heading = 0.01;
    public static double kI_heading = 0;
    public static double kD_heading = 0;

    // 可在FTC Dashboard上调整的目标角度（度）
    public static double targetHeadingDegrees = 90;

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private NewMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NewMecanumDrive(hardwareMap);

        // 初始化驱动并等待开始
        drive.setPoseEstimate(new Pose2d());
        drive.update();
        waitForStart();

        double integralSumHeading = 0;
        double lastErrorHeading = 0;

        while (opModeIsActive()) {
            // 获取当前位姿
            Pose2d currentPos = drive.getPoseEstimate();
            double currentHeading = currentPos.getHeading();

            // 将目标角度转换为弧度
            double targetHeading = Math.toRadians(targetHeadingDegrees);

            // 计算误差
            double errorHeading = targetHeading - currentHeading;

            // 误差积分
            integralSumHeading += errorHeading;

            // 误差微分
            double derivativeHeading = errorHeading - lastErrorHeading;

            // 计算PID输出
            double pidOutput = kP_heading * errorHeading + kI_heading * integralSumHeading + kD_heading * derivativeHeading;

            // 应用PID输出进行自转
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            pidOutput
                    )
            );

            // 更新驱动状态
            drive.update();

            // 在FTC Dashboard上显示相关信息
            telemetry_M.addData("Target Heading (deg)", targetHeadingDegrees);
            telemetry_M.addData("Current Heading (rad)", currentHeading);
            telemetry_M.addData("Heading Error (rad)", errorHeading);
            telemetry_M.addData("kP_heading", kP_heading);
            telemetry_M.addData("kI_heading", kI_heading);
            telemetry_M.addData("kD_heading", kD_heading);
            telemetry_M.update();

            // 更新上一次的误差
            lastErrorHeading = errorHeading;
        }
    }
}