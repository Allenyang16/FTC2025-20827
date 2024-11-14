package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

import java.util.Locale;

@TeleOp
@Disabled
public class TeleOp20827 extends LinearOpMode {
    Runnable update;
    enum Sequence{
        RUN, RELEASE
    }
    private Sequence sequence;

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure upper = new SuperStructure(this);
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        sequence = Sequence.RUN;

        update = ()->{
            drive.update();
            upper.update();
            XCYBoolean.bulkRead();
            telemetry.update();
        };
        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        XCYBoolean grab = new XCYBoolean(()-> gamepad1.right_bumper);
        XCYBoolean toOrigin = new XCYBoolean(()-> gamepad1.left_stick_button);
        XCYBoolean intakeToOrigin = new XCYBoolean(()-> gamepad1.right_stick_button);
        XCYBoolean toHighRelease = new XCYBoolean(()-> gamepad1.dpad_up);
        XCYBoolean downWrist = new XCYBoolean(()-> gamepad1.left_bumper);
        XCYBoolean resetHeading = new XCYBoolean(()-> gamepad1.back);

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad2.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad2.a);
        XCYBoolean toReleaseHighChamber = new XCYBoolean(()-> gamepad2.y);
        XCYBoolean toReleaseLowChamber = new XCYBoolean(()-> gamepad2.a);

        upper.initialize();
        drive.setPoseEstimate(new Pose2d(12,-52,Math.toRadians(90)));
        waitForStart();

        while (opModeIsActive()){

            if(resetHeading.toTrue()){
                drive.resetHeading();
            }

            if(sequence == Sequence.RUN){
                if(intakeFar.toTrue()){
                    upper.setWristIntake_ParallelToGround();
                    upper.setSpinwristIntake();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setClawOpen();
                    delay(1000);
                    upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                }
                if(intakeNear.toTrue()){
                    upper.setWristIntake_ParallelToGround();
                    upper.setSpinwristIntake();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setClawOpen();
                    delay(1000);
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    delay(500);

                }
                if(grab.toTrue()){
                    upper.switchClawState();
                }
                if(downWrist.toTrue()){
                    upper.setWristIntake();
                }
                if(toOrigin.toTrue()){
                    upper.setWristIntake_ParallelToGround();
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    delay(1500);
                    upper.setArmPosition(SuperStructure.ARM_RELEASE);
//                    upper.setWristIntake();
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    sequence = Sequence.RELEASE;
                }
                if(intakeToOrigin.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                    upper.setWristIntake_ParallelToGround();
                }
            }

            if(sequence == Sequence.RELEASE){
                if(toHighRelease.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_RELEASE);
                    upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
                    upper.setWristReleaseBox();
                    delay(1500);
                }
                if(grab.toTrue()){
                    upper.switchClawState();
                    sequence = Sequence.RUN;
                }
            }


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("Position", data);
            telemetry.addData("Sequence", sequence);

            drive.setGlobalPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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
