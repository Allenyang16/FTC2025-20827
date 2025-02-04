package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
@Config
public class Turn_Pid_Test extends LinearOpMode {
    private int ct = 200;
    public static double kP_heading = 0.01;
    public static double kI_heading = 0;
    public static double kD_heading = 0;

    public static double kP_xy = 0.01;
    public static double kI_xy = 0;
    public static double kD_xy = 0;

    public static double t_x = 0;
    public static double t_y = 0;
    public static double t_heading = 0;

    public static double t1_x = 0;
    public static double t1_y = 0;
    public static double t1_heading = -90;

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private Pose2d currentPos = new Pose2d(t_x, t_y,Math.toRadians(t_heading));
    private Pose2d targetPos = new Pose2d(t1_x, t1_y, Math.toRadians(t1_heading));
    private Pose2d tragetPosWithouChangeDirection = new Pose2d(t1_x,t1_y , t_heading);
    private Pose2d targetPosWithoutChangePosition = new Pose2d(t_x,t_y , Math.toRadians(t1_heading));
    private NewMecanumDrive drive;
    private SuperStructure superstructure;

    @Override
    public void runOpMode() throws InterruptedException {
        superstructure = new SuperStructure(this);
        drive = new NewMecanumDrive(hardwareMap);

        XCYBoolean moveWithoutChangeDirection = new XCYBoolean(() -> gamepad1.dpad_up);
        XCYBoolean toOriginalPos = new XCYBoolean(() -> gamepad1.dpad_down);
        XCYBoolean moveWithChangeDirection = new XCYBoolean(() -> gamepad1.dpad_left);
        XCYBoolean changeDirection = new XCYBoolean(() -> gamepad1.dpad_right);

        double integralSumX = 0;
        double integralSumY = 0;
        double integralSumHeading = 0;

        double lastErrorX = 0;
        double lastErrorY = 0;
        double lastErrorHeading = 0;

        Runnable update = () -> {
            drive.update();
            superstructure.update();
            XCYBoolean.bulkRead();
            telemetry_M.update();
        };

        drive.setUpdateRunnable(update);
        superstructure.setUpdateRunnable(update);
        superstructure.initialize();

        drive.setPoseEstimate(new Pose2d(t1_x, t1_y, Math.toRadians(t1_heading)));
        drive.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.setSimpleMoveTolerance(0.1, 0.1, Math.toRadians(3));
            drive.setSimpleMovePower(1);
            if (moveWithoutChangeDirection.toTrue()) {
                drive.moveTo(tragetPosWithouChangeDirection, ct);
            }

            if (moveWithChangeDirection.toTrue()) {
                drive.moveTo(targetPos, ct);
            }
            if (toOriginalPos.toTrue()) {
                drive.moveTo(currentPos, ct);
            }
            if (changeDirection.toTrue()) {
                drive.moveTo(targetPosWithoutChangePosition, ct);
            }

            // 获取当前位姿
            Pose2d currentPos = drive.getPoseEstimate();


        }       }
}
