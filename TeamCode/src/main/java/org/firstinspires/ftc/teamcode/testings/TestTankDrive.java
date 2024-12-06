package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
public class TestTankDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superStructure = new SuperStructure(this);
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        superStructure.initialize();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Define movement inputs using the D-pad for translational control
            double y = 0;
            double x = 0;

            // Use the D-pad for forward/backward (y) and left/right (x) movement
            if (gamepad1.dpad_up) {
                y = 1;  // Forward
            } else if (gamepad1.dpad_down) {
                y = -1; // Backward
            }

            if (gamepad1.dpad_left) {
                x = -1; // Left
            } else if (gamepad1.dpad_right) {
                x = 1;  // Right
            }

            // Use the right stick for rotation (rx)
            double rx = gamepad1.right_stick_x;

            // Denominator ensures we maintain the same ratio of motor power while staying within the [-1, 1] range
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}