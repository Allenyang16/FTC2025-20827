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
public class TestArmPID extends LinearOpMode {
//    public static int referenceAngle = 45;
    public static int position = 200;
    public static double power = 0.9;
//    public static double kS = 0;
    public static double kCos = 0;
//    public static double kV = 0;
//    public static double kA = 0;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(this);
        NewMecanumDrive drive =new NewMecanumDrive(hardwareMap);

        //XCYBoolean testMove = new XCYBoolean(()->gamepad1.b);
        XCYBoolean testArm = new XCYBoolean(()->gamepad1.a);
        XCYBoolean armBack = new XCYBoolean(()->gamepad1.b);

//        superstructure.resetSlide();
//        superstructure.setSlidePosition(0);

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


        waitForStart();

        while (opModeIsActive()) {
            if(testArm.toTrue()){
                superstructure.setArmPosition(position);
            }
            if(armBack.toTrue()){
                superstructure.setArmPosition(0);
            }
            telemetry_M.addData("arm:", superstructure.getArmPosition());
            telemetry.addData("slideLeft: ",superstructure.getSlideLeftPosition());
            telemetry.addData("slideRight: ",superstructure.getSlideRightPosition());
            telemetry_M.addData("armPower: ",superstructure.getArmPower());
            telemetry_M.addData("Arm Error",superstructure.getArmPosition() - superstructure.getArmTargetPosition());

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
