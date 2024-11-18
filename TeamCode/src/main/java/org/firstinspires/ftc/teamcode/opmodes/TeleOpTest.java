package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

import java.util.Locale;

@TeleOp
public class TeleOpTest extends LinearOpMode {
    Runnable update;
    enum Sequence{
        RUN, RELEASE
    }
    enum IntakeState{
        FAR, NEAR, POST
    }
    private Sequence sequence;
    private IntakeState intakeState;

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
            // TODO: CHECK WHETHER THIS CAN WORK
            drive.setGlobalPower(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.5 * gamepad1.right_stick_x);
        };
        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad1.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad1.a);

        XCYBoolean toReleaseHighChamber = new XCYBoolean(()-> sequence == Sequence.RELEASE && gamepad1.y);
        XCYBoolean toReleaseLowChamber = new XCYBoolean(()-> sequence == Sequence.RELEASE && gamepad1.a);

        XCYBoolean grab = new XCYBoolean(()-> gamepad1.right_bumper);
        XCYBoolean toOrigin = new XCYBoolean(()-> intakeState == IntakeState.POST && gamepad1.left_stick_button);
        XCYBoolean intakeToOrigin = new XCYBoolean(()-> gamepad1.right_stick_button);
        XCYBoolean toHighRelease = new XCYBoolean(()-> gamepad1.dpad_up);
        XCYBoolean downWrist = new XCYBoolean(()-> gamepad1.left_bumper);
        XCYBoolean resetHeading = new XCYBoolean(()-> gamepad1.back);

        upper.initialize();
        drive.setPoseEstimate(new Pose2d(12,-52,Math.toRadians(90)));
        intakeState = IntakeState.NEAR;
        waitForStart();

        while (opModeIsActive()){
            Pose2d current_pos = drive.getPoseEstimate();

            if(resetHeading.toTrue()){
                drive.resetHeading();
            }

            if(sequence == Sequence.RUN){
                upper.setArmPosition(SuperStructure.ARM_INTAKE);

                if(downWrist.toTrue()){
                    upper.setWristIntake();
                }
                if(intakeFar.toTrue()){
                    if(intakeState == IntakeState.NEAR){
                        upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                        intakeState = IntakeState.FAR;
                    }else {
                        upper.setWristIntake_ParallelToGround();
                        upper.setSpinwristIntake();
                        upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                        upper.setArmPosition(SuperStructure.ARM_INTAKE);
                        upper.setClawOpen();
                        delay(500);
                        upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                        intakeState = IntakeState.FAR;
                    }
                }

                if(intakeNear.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);

                    upper.setWristIntake_ParallelToGround();
                    upper.setSpinwristIntake();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setClawOpen();
                    delay(500);

                    intakeState = IntakeState.NEAR;
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                }


                if(intakeToOrigin.toTrue()){
                    if(intakeState == IntakeState.FAR){
                        upper.setWristIntake_ParallelToGround();
                        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                        delay(500);
                    }else{
                        upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                        upper.setWristIntake_ParallelToGround();
                    }
                    intakeState = IntakeState.POST;
                }

                if(toOrigin.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_RELEASE);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    sequence = Sequence.RELEASE;
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
                    delay(150);
                    sequence = Sequence.RUN;

                    upper.setArmPosition(100);
                    delay(500);
                    upper.setSlidePosition(0);
                    delay(500);
                }

                if(toReleaseHighChamber.toTrue()){

                }
            }


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", current_pos.getX(), current_pos.getY(), Math.toDegrees(current_pos.getHeading()));
            telemetry.addData("Position: ", data);
            telemetry.addData("Sequence: ", sequence);
            telemetry.addData("rightSlide_encoder: ", upper.getSlideRightPosition());

            drive.setGlobalPower(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.5 * gamepad1.right_stick_x);

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
