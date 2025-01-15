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
public class TestTranslationPID extends LinearOpMode {
    public static int position = 200;
    public static double power = 0.9;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private Pose2d forwardPos = new Pose2d(-48,48,0);
    public static double t_x = 0.5, t_y = 0.5, t_heading = 3;
    public static double t1_x = AutoMaster.chamber_x, t1_y = AutoMaster.chamber_y, t1_heading = -90;
    public static double t2_x = AutoMaster.intakeSpecimen_x, t2_y = AutoMaster.pre_intakeSpecimen_y, t2_heading = 90;
    public static double t3_x = t2_x, t3_y = AutoMaster.intakeSpecimen_y;
    private Pose2d backPos = new Pose2d(48,48,0);
    private Pose2d rightPos = new Pose2d(48,48,Math.toRadians(-90));
    private Pose2d leftPos = new Pose2d(-48,48, Math.toRadians(-90));

    private Pose2d targetPos = new Pose2d(9,62,Math.toRadians(-90));
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

        XCYBoolean toTargetPos1 = new XCYBoolean(()-> gamepad1.a);


        Pose2d targetPos1 = new Pose2d(t1_x,t1_y,Math.toRadians(t1_heading));
        Pose2d targetPos2 = new Pose2d(t2_x,t2_y,Math.toRadians(t1_heading));
        Pose2d targetPos3 = new Pose2d(t3_x,t3_y,Math.toRadians(t1_heading));

        Runnable update = ()->{
            drive.update();
            superstructure.update();
            XCYBoolean.bulkRead();
            telemetry.update();
        };

        drive.setUpdateRunnable(update);
        superstructure.setUpdateRunnable(update);
        superstructure.initialize();

        drive.setPoseEstimate(new Pose2d(-9,62,Math.toRadians(-90)));
        drive.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.setSimpleMoveTolerance(t_x,t_y,Math.toRadians(t_heading));
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
                drive.moveTo(leftPos,correctTime);
            }

            if(toTargetPos1.toTrue()){
                targetPos = targetPos1;
                drive.moveTo(targetPos1,correctTime);
            }
            if(gamepad1.b){
                targetPos = targetPos2;
                drive.moveTo(targetPos2,correctTime);
            }
            if(gamepad1.y){
                targetPos = targetPos3;
                drive.moveTo(targetPos3,correctTime);
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
