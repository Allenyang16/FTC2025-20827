package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

@TeleOp (group = "Testing")
@Config
public class TestLocalizer extends LinearOpMode {
    //NewMecanumDrive drive = new NewMecanumDrive();
    NewMecanumDrive drive;

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static  double x = -15, y = -62, heading = 90;
    public static  double target_x = -57, target_y = -57, target_heading = 45;
    private static Pose2d startPos;
    private static Pose2d targetPos = new Pose2d(0,0,0);
    @Override
    public void runOpMode(){
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        XCYBoolean testMove = new XCYBoolean(()-> gamepad1.b);
        drive = new NewMecanumDrive(hardwareMap);
        Runnable update = ()->{
            drive.update();
            XCYBoolean.bulkRead();
        };

        drive.setUpdateRunnable(update);

        //drive.setUp(hardwareMap);
        startPos = new Pose2d(x,y,Math.toRadians(heading));
        drive.setPoseEstimate(startPos);
        drive.setSimpleMovePower(0.95);
        drive.update();
        telemetry.addData("Pos Estimate: ",drive.getPoseEstimate());
        telemetry.update();



        waitForStart();

        while (opModeIsActive()){
            double standard_xPos = drive.getPoseEstimate().getX();
            double standard_yPos = drive.getPoseEstimate().getY();

//            if(testMove.toTrue()){
//                drive.initSimpleMove(new Pose2d(24,0,Math.toRadians(0)));
//            }
//            if (testMove.toFalse()){
//                drive.stopTrajectory();
//                drive.setMotorPowers(
//                        0,0,0,0
//                );
//            }
            if(gamepad1.a){
                drive.moveTo(new Pose2d(target_x,target_y,Math.toRadians(target_heading)),AutoMaster.correcting_time);
                targetPos = new Pose2d(target_x,target_y,Math.toRadians(target_heading));
            }
            if(gamepad1.b){
                drive.stopTrajectory();
                drive.setMotorPowers(
                        0,0,0,0
                );
            }
            if(gamepad1.x){
                drive.moveTo(startPos,0);
                targetPos = startPos;
            }
            Pose2d error = drive.getPoseEstimate().minus(targetPos);

            telemetry.addData("Current X Position (in): ", "%.3f", standard_xPos);
            telemetry.addData("Current Y Position (in): ", "%.3f", standard_yPos);
            telemetry.addData("Current Heading: ", drive.getPoseEstimate().getHeading());

            telemetry.addData("Error: ", error);
            telemetry.addData("boolean", drive.isBusy());
            telemetry.addData("power", leftFront.getPower());
            telemetry.update();
            update.run();
        }
    }
}
