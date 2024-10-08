package org.firstinspires.ftc.teamcode.FTC16093;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "my motor test")
@Config
public class MyMotorTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public static double max_power = 1;


    public static DcMotorEx[] motorObjects=new DcMotorEx[10];
    public static String[] motorNames={"front_left","front_right","rear_left","rear_right"};


    @Override
    public void runOpMode() {

        for(int idx=0;idx< motorNames.length;idx++) {//init
            motorObjects[idx] = hardwareMap.get(DcMotorEx.class,motorNames[idx] );
        }
        while (opModeIsActive()) {
            for(int idx=0;idx< motorNames.length;idx++) {
                DcMotorEx motor=motorObjects[idx];
                String name=motorNames[idx];
                motor.setPower(max_power);
                telemetry_M.addData("is busy"+name, motor.isBusy());
                telemetry_M.addData("encoder"+name, motor.getCurrentPosition());
                telemetry_M.addData("velocity"+name, motor.getVelocity());
            }
            telemetry_M.update();
        }
    }
}