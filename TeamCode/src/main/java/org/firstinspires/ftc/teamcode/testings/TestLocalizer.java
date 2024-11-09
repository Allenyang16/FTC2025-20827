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

@TeleOp (group = "Testing")
@Config
public class TestLocalizer extends LinearOpMode {
    //NewMecanumDrive drive = new NewMecanumDrive();
    NewMecanumDrive drive;

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static  double x = 48, y = 0, heading = 0;
    private static Pose2d startPos;
    @Override
    public void runOpMode(){
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
                drive.initSimpleMove(new Pose2d(48,24,Math.toRadians(0)));
            }
            if(gamepad1.b){
                drive.stopTrajectory();
                drive.setMotorPowers(
                        0,0,0,0
                );
            }
            Pose2d error = drive.getLastError();
            telemetry.addData("Current X Position (in): ", "%.3f", standard_xPos);
            telemetry.addData("Current Y Position (in): ", "%.3f", standard_yPos);
            telemetry.addData("Current Heading: ", drive.getPoseEstimate().getHeading());

            telemetry.addData("Error: ", error);
            telemetry.update();
            update.run();
        }
    }
}
