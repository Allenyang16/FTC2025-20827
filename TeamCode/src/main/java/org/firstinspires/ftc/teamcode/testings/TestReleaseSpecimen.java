package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
public class TestReleaseSpecimen extends LinearOpMode {
    Runnable update;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure upper = new SuperStructure(this);
        XCYBoolean up = new XCYBoolean(()-> gamepad1.dpad_up);
        XCYBoolean down = new XCYBoolean(()->gamepad1.dpad_down);
        XCYBoolean grab = new XCYBoolean(()-> gamepad1.a);

        update = ()->{
            upper.update();
            XCYBoolean.bulkRead();
            telemetry.update();
            telemetry_M.update();
        };

        upper.setUpdateRunnable(update);
        upper.initialize();
        waitForStart();

        while (opModeIsActive()){
            if(up.toTrue()){
                upper.setWristIntake();
                upper.setClawGrab();
                upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH);
                upper.setArmPosition(SuperStructure.ARM_CHAMBER);
            }
            if(down.toTrue()){
                upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH - SuperStructure.SLIDE_CHAMBER_DELTA);
            }
            if(grab.toTrue()){
                upper.switchClawState();
            }

            update.run();
        }

    }
}
