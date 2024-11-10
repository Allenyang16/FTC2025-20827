package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
public class TestClaw extends LinearOpMode {
    Runnable update;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure upper = new SuperStructure(this);
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
            if(grab.toTrue()){
                upper.switchClawState();
            }
            if (gamepad1.b){
                upper.switchClawState();
            }
            update.run();
        }

    }
}
