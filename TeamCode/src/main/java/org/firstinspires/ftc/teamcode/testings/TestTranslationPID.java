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
public class TestTranslationPID extends LinearOpMode {
    public static int position = 200;
    public static double power = 0.9;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private Pose2d forwardPos = new Pose2d(48,-48,0);
    private Pose2d backPos = new Pose2d(-48,-48,0);
    private Pose2d rightPos = new Pose2d(48,-48,Math.toRadians(90));
    private Pose2d leftPos = new Pose2d(-48,-48, Math.toRadians(90));

    private Pose2d targetPos = new Pose2d(0,-48,0);
    public static int correctTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);
        NewMecanumDrive drive =new NewMecanumDrive(hardwareMap);

        XCYBoolean moveForward = new XCYBoolean(()->gamepad1.dpad_up);
        XCYBoolean moveBack = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean strafeRight = new XCYBoolean(()->gamepad1.dpad_right);
        XCYBoolean strafeLeft = new XCYBoolean(()->gamepad1.dpad_left);
        XCYBoolean turnClockwise = new XCYBoolean(()->gamepad1.right_bumper);
        XCYBoolean turnCounterClockwise = new XCYBoolean(()-> gamepad1.left_bumper);


        Runnable update = ()->{
            drive.update();
            superstructure.update();
            XCYBoolean.bulkRead();
            telemetry.update();
        };

        drive.setUpdateRunnable(update);
        superstructure.setUpdateRunnable(update);
        superstructure.initialize();

        drive.setPoseEstimate(new Pose2d(0,-48,0));
        drive.update();
        waitForStart();

        while (opModeIsActive()) {
            Pose2d currentPos = drive.getPoseEstimate();
            if(moveForward.toTrue()){
                targetPos = forwardPos;
                drive.moveTo(forwardPos, correctTime);
            }
            if(moveBack.toTrue()){
                targetPos = backPos;
                drive.moveTo(backPos,correctTime);
            }
            if(strafeRight.toTrue()){
                targetPos = rightPos;
                drive.moveTo(rightPos,correctTime);
            }
            if(strafeLeft.toTrue()){
                targetPos = leftPos;
                drive.moveTo(leftPos, correctTime);
            }


            telemetry_M.addData("X Error", targetPos.getX() - drive.getPoseEstimate().getX());
            telemetry_M.addData("Y Error", targetPos.getY() - drive.getPoseEstimate().getY());
            telemetry_M.addData("Heading Error", Math.toDegrees(targetPos.getHeading() - drive.getPoseEstimate().getHeading()));
            update.run();
        }
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }

}
