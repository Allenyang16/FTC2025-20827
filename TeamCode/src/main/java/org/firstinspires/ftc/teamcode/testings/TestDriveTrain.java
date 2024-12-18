package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
@Config
public class TestDriveTrain extends LinearOpMode {
    public static int position = 200;
    public static double power = 0.9;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        NewMecanumDrive drive =new NewMecanumDrive(hardwareMap);

        Runnable update = ()->{
            drive.update();
            XCYBoolean.bulkRead();
            telemetry.update();
        };

        drive.setUpdateRunnable(update);

        drive.setPoseEstimate(new Pose2d(39,-62,Math.toRadians(90)));
        drive.update();
        waitForStart();

        while (opModeIsActive()) {
            Pose2d currentPos = drive.getPoseEstimate();
            drive.setGlobalPower(gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x);
            update.run();
        }
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }

}
