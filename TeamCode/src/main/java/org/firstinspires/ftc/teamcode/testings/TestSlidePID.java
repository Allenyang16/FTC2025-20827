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
public class TestSlidePID extends LinearOpMode {
    public static double x = 12;
    public static double y = 0, heading = 0;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);
        NewMecanumDrive drive =new NewMecanumDrive(hardwareMap);

        XCYBoolean a = new XCYBoolean(()->gamepad1.a);
        XCYBoolean b = new XCYBoolean(()->gamepad1.b);
        XCYBoolean y = new XCYBoolean(()->gamepad1.y);
        XCYBoolean changeSlideState = new XCYBoolean(()-> gamepad1.dpad_down);

        Runnable update = ()->{
            drive.update();
            superstructure.update();
            XCYBoolean.bulkRead();
            telemetry.update();
        };

        drive.setUpdateRunnable(update);
        superstructure.setUpdateRunnable(update);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.update();
        superstructure.resetSlide();
        waitForStart();

        while (opModeIsActive()) {
            if (a.toTrue()) {
                superstructure.setSlidePosition(SuperStructure.SLIDE_MIN);
            }
            if(y.toTrue()) {
                superstructure.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
            }
            if(b.toTrue()) {
                superstructure.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
            }
            if(changeSlideState.toTrue()){
                superstructure.switchSlideState();
            }


            telemetry_M.addData("Slide Left Power: ", superstructure.getSlideLeftPower());
            telemetry_M.addData("Slide Right Power: ", superstructure.getSlideRightPower());

            telemetry_M.addData("slideL: ",superstructure.getSlideLeftPosition());
            telemetry_M.addData("slideR: ",superstructure.getSlideRightPosition());
            telemetry_M.addData("SlideL Error",superstructure.getSlideLeftPosition() - superstructure.getSlideTargetPosition());
            telemetry_M.addData("SlideR Error",superstructure.getSlideRightPosition() - superstructure.getSlideTargetPosition());
            telemetry_M.update();
            update.run();
        }
    }

    private void drive_period() {
        //there's nothing here
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }

}
