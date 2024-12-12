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
import org.firstinspires.ftc.teamcode.XCYBoolean;

@TeleOp (group = "Testing")
@Config
public class SlideTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;
    public static int encoder_position = 1150;
    public static double max_power = 1;
    public static boolean read_only = true;
    public static boolean reverse_left = true;
    public static boolean reverse_right = false;
    public static boolean set_power_mode_or_set_position_mode = false;
    public static boolean reset = true;

    @Override
    public void runOpMode(){
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");
        XCYBoolean enablePowerMode = new XCYBoolean(()->gamepad1.a);
        XCYBoolean disablePowerMode = new XCYBoolean(()-> gamepad1.b);

        waitForStart();
        if (reset) {
            mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (reverse_left) {
            mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (reverse_right) {
            mSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        while(opModeIsActive()){
            if(enablePowerMode.toTrue()){
                set_power_mode_or_set_position_mode = true;
                read_only = false;
            }
            if(disablePowerMode.toTrue()){
                set_power_mode_or_set_position_mode = false;
                read_only = true;
            }

            if (set_power_mode_or_set_position_mode) {
                if (read_only) {
                    mSlideRight.setPower(0);
                    mSlideLeft.setPower(0);
                }
                else {
                    mSlideLeft.setPower(-gamepad1.right_stick_y);
                    mSlideRight.setPower(-gamepad1.right_stick_y);
                }
                mSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            } else {
                if (!read_only) {
                    mSlideRight.setTargetPosition(encoder_position);
                    mSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mSlideRight.setPower(max_power);

                    mSlideLeft.setTargetPosition(encoder_position);
                    mSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mSlideLeft.setPower(max_power);
                }
                telemetry_M.addData("is busy_leftSlide", mSlideLeft.isBusy());
                telemetry_M.addData("is busy_rightSlide", mSlideRight.isBusy());
            }


            telemetry_M.addData("encoder_slideLeft", mSlideLeft.getCurrentPosition());
            telemetry_M.addData("encoder_slideRight", mSlideRight.getCurrentPosition());

            telemetry_M.addData("right_velocity", mSlideRight.getVelocity());
            telemetry_M.addData("left_velocity", mSlideLeft.getVelocity());
            telemetry_M.addData("power: ",-gamepad1.right_stick_y);
            telemetry_M.update();
        }
    }
}
