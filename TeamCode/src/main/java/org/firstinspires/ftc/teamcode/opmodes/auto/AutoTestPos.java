package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;

@Autonomous
public class AutoTestPos extends AutoMaster {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private DcMotorEx leftFront;

    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = NEGATIVE;
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");

        // TODO: Check the init of positions
        initHardware();
        XCYBoolean toDrop = new XCYBoolean(()-> gamepad1.a);
        XCYBoolean toIntake1 = new XCYBoolean(()-> gamepad1.b);
        XCYBoolean toIntake2 = new XCYBoolean(()-> gamepad1.y);
        XCYBoolean toIntake3 = new XCYBoolean(()-> gamepad1.x);

        while(opModeInInit()){

        }
        waitForStart();
        while(opModeIsActive()){
            if(toDrop.toTrue()){
                moveToDrop();
            }
            if(toIntake1.toTrue()){
                moveToIntake1();
            }
            if(toIntake2.toTrue()){
                moveToIntake2();
            }
            if(toIntake3.toTrue()){
                moveToIntake3();
            }

            telemetry_M.addData("is at correct pos: ", isAtCorrectPos());
            telemetry_M.addData("power: ", getMotorPower());
            telemetry_M.addData("LeftFront velocity: ", leftFront.getVelocity());
            telemetry_M.update();
            XCYBoolean.bulkRead();
        }




    }
}
