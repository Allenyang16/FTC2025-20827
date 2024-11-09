package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

import java.util.Locale;

@TeleOp
public class TeleOpTest extends LinearOpMode {
    Runnable update;

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure upper = new SuperStructure(this);
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);

        update = ()->{
            drive.update();
            upper.update();
            XCYBoolean.bulkRead();
            telemetry.update();
        };
        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad1.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad1.a);
        XCYBoolean grab = new XCYBoolean(()-> gamepad1.right_bumper);
        XCYBoolean toOrigin = new XCYBoolean(()-> gamepad1.left_stick_button);
        XCYBoolean toHighRelease = new XCYBoolean(()-> gamepad1.dpad_up);
        XCYBoolean openClaw = new XCYBoolean(()-> gamepad1.left_bumper);

        upper.initialize();
        drive.setPoseEstimate(new Pose2d(12,-52,Math.toRadians(90)));
        waitForStart();

        while (opModeIsActive()){

            if(intakeFar.toTrue()){
                upper.setArmPosition(SuperStructure.ARM_INTAKE);
                upper.setClawOpen();
                delay(1000);
                upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                // miniArm
                upper.setWristIntake();

            }

            if(intakeNear.toTrue()){
                upper.setArmPosition(SuperStructure.ARM_INTAKE);
                upper.setClawOpen();
                delay(1000);
                upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                delay(500);
                upper.setWristIntake();
            }

            if(grab.toTrue()){
                upper.setClawGrab();
            }
            if(openClaw.toTrue()){
                upper.setClawOpen();
            }

            if(toOrigin.toTrue()){
                upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                delay(1000);
                upper.setArmPosition(SuperStructure.ARM_RELEASE);
                upper.setWristIntake();
            }

            if(toHighRelease.toTrue()){
                upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
                upper.setWristReleaseBox();
            }

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());
            telemetry.addData("Position", data);

            drive.setGlobalPower(-gamepad1.left_stick_y,-gamepad1.left_stick_x, 0.3 * gamepad1.right_stick_x);
            update.run();
        }
    }

    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}
