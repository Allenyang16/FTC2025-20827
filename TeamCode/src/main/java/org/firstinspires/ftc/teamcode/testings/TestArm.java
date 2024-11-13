package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (group = "Testing")
@Config
public class TestArm extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private DcMotorEx mArmLeft = null;
    private DcMotorEx mArmRight = null;

    public static int encoder_position = 1150;
    public static double max_power = 1;
    public static boolean read_only = true;
    public static boolean reverse_left = true;
    public static boolean reverse_right = false;

    public static boolean set_power_mode_or_set_position_mode = false;

    public static boolean reset = true;

    @Override
    public void runOpMode(){
        mArmLeft = hardwareMap.get(DcMotorEx.class,"armLeft");
        mArmRight = hardwareMap.get(DcMotorEx.class,"armRight");

        waitForStart();
        if (reset) {
            mArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (reverse_left) {
            mArmLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (reverse_right) {
            mArmRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        while(opModeIsActive()){
            if (set_power_mode_or_set_position_mode) {
                if (read_only) {
                    mArmRight.setPower(0);
                    mArmLeft.setPower(0);
                }
                else {
                    mArmLeft.setPower(-gamepad1.right_stick_y);
                    mArmRight.setPower(-gamepad1.right_stick_y);
                }
                mArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            } else {
                if (!read_only) {
                    mArmRight.setTargetPosition(encoder_position);
                    mArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mArmRight.setPower(max_power);

                    mArmLeft.setTargetPosition(encoder_position);
                    mArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mArmLeft.setPower(max_power);
                    sleep(10000);
                }
                telemetry_M.addData("is busy_leftArm", mArmLeft.isBusy());
                telemetry_M.addData("is busy_rightArm", mArmRight.isBusy());
            }


            telemetry_M.addData("encoder_ArmLeft", mArmLeft.getCurrentPosition());
            telemetry_M.addData("encoder_ArmRight", mArmRight.getCurrentPosition());

            telemetry_M.addData("right_velocity", mArmRight.getVelocity());
            telemetry_M.addData("left_velocity", mArmLeft.getVelocity());
            telemetry_M.addData("Current power: ", -gamepad1.right_stick_y);
            telemetry_M.update();
        }
    }
}
