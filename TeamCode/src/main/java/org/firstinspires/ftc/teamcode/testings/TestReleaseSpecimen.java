package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
public class TestReleaseSpecimen extends LinearOpMode {
    Runnable update;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure upper = new SuperStructure(this);
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);

        XCYBoolean up = new XCYBoolean(()-> gamepad1.dpad_up);
        XCYBoolean down = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean grab = new XCYBoolean(()-> gamepad1.right_bumper);
        XCYBoolean toIntakePos = new XCYBoolean(()-> gamepad1.a);

        drive.setPoseEstimate(AutoMaster.endPos);
        drive.update();

        update = ()->{
            upper.update();
            drive.update();
            XCYBoolean.bulkRead();
            telemetry.update();
            telemetry_M.update();
            drive.setGlobalPower(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.5 * gamepad1.right_stick_x);
        };

        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        upper.initialize();
        waitForStart();

        while (opModeIsActive()){
            if(toIntakePos.toTrue()){
                upper.setSpinWristIntake();
                upper.setWristIntakeSpecimen();
                upper.setClawOpen();
                upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
            }

            if(up.toTrue()){
                upper.setWristIntake();
                upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
                upper.setClawGrab();
                upper.setSpinWristRelease_specimen();

                upper.setSlidePosition_verticle(SuperStructure.SLIDE_CHAMBER_HIGH);
            }
            if(down.toTrue()){
                upper.setSlidePosition_verticle(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
            }
            if(grab.toTrue()){
                upper.switchClawState();
            }


            update.run();
        }

    }
}
