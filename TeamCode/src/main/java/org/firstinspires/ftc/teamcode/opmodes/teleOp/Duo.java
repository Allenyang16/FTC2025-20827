package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

import java.util.Locale;

@TeleOp (name = "TeleOp_Duo")
public class Duo extends LinearOpMode {
    Runnable update;
    enum Sequence{
        RUN, INTAKE_SAMPLE, INTAKE_SPECIMEN, RELEASE_SAMPLE,RELEASE_SPECIMEN
    }
    enum IntakeState{
        FAR, NEAR, POST, SPECIMEN
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
            drive.setGlobalPower(upper.translation_coefficient() * gamepad1.left_stick_y, upper.translation_coefficient() * gamepad1.left_stick_x, upper.heading_coefficient() * gamepad1.right_stick_x);
        };
        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        XCYBoolean grab = new XCYBoolean(()-> gamepad2.right_bumper);

        XCYBoolean resetUpper = new XCYBoolean(()-> gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0);
        XCYBoolean resetHeading = new XCYBoolean(()-> gamepad1.a);
        XCYBoolean toOrigin = new XCYBoolean(()-> (intakeState == IntakeState.POST || intakeState == IntakeState.SPECIMEN) && gamepad1.left_stick_button);
        XCYBoolean toPostIntake = new XCYBoolean(()-> (intakeState != IntakeState.SPECIMEN) && gamepad1.right_stick_button);

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad2.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad2.a);
        XCYBoolean toIntakeSpecimen = new XCYBoolean(()->gamepad2.b);
        XCYBoolean toHighRelease_sample = new XCYBoolean(()-> gamepad2.dpad_up);
        XCYBoolean downWrist = new XCYBoolean(()-> gamepad2.left_bumper);
        XCYBoolean spinWristClockwise = new XCYBoolean(()-> gamepad2.right_trigger > 0);
        XCYBoolean spinWristCounterClockwise = new XCYBoolean(()-> gamepad2.left_trigger > 0);
        XCYBoolean toReleaseHighChamber = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_up);
        XCYBoolean toPullDownSpecimen = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_down);
        XCYBoolean hang = new XCYBoolean(()-> gamepad2.x);

        upper.initialize();
        drive.setPoseEstimate(AutoMaster.endPos);
        intakeState = IntakeState.NEAR;
        waitForStart();

        // TODO: try to remove as much delay as possible
        while (opModeIsActive()){
            Pose2d current_pos = drive.getPoseEstimate();

            if(resetUpper.toTrue()){
                upper.setArmPosition(SuperStructure.ARM_INTAKE);
                upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                upper.setClawOpen();
                upper.setSpinWristIntake();
                upper.setWristPreIntake();
                upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);
                sequence = Sequence.RUN;
            }

            if(resetHeading.toTrue()){
                drive.resetHeading();
            }

            // TODO: TEST this
            if(hang.toTrue()){
                upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
                upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
                delay(5000);
                upper.setSlidePosition(SuperStructure.SLIDE_MIN);
            }

            if(upper.getSlidePosition() < SuperStructure.SLIDE_INTAKE_MAX){

            }

            if(sequence == Sequence.RUN){
                if(intakeFar.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);
                    upper.setClawOpen();
                    delay(200);
                    upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                    intakeState = IntakeState.FAR;
                    sequence = Sequence.INTAKE_SAMPLE;
                }

                if(intakeNear.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    upper.setSpinWristIntake();
                    upper.setWristPreIntake();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setClawOpen();
                    intakeState = IntakeState.NEAR;
                    sequence = Sequence.INTAKE_SAMPLE;
                }

                if(toIntakeSpecimen.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    upper.setWristIntakeSpecimen();
                    upper.setSpinWristIntake_specimen();
                    upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    intakeState = IntakeState.SPECIMEN;
                    sequence = Sequence.INTAKE_SPECIMEN;
                }

                if(toHighRelease_sample.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
                    upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
                    upper.setWristReleaseBox();
                    sequence = Sequence.RELEASE_SAMPLE;
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                }
            }

            if(sequence == Sequence.INTAKE_SAMPLE){
                if(grab.toTrue()){
                    upper.switchClawState();
                }
                if(downWrist.toTrue()){
                    upper.switchWristIntakeState();
                }
                if(spinWristClockwise.toTrue()){
                    upper.setSpinWristIntake_spinClockwise();
                }
                if(spinWristCounterClockwise.toTrue()){
                    upper.setSpinWristIntake_spinCounterClockwise();
                }

                if(intakeFar.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                    upper.setClawOpen();
                    intakeState = IntakeState.FAR;
                }
                if(intakeNear.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    upper.setClawOpen();
                    intakeState = IntakeState.NEAR;
                }

                if(toPostIntake.toTrue()){
                    upper.setWristPreIntake();
                    if(intakeState == IntakeState.FAR){
                        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    }else{
                        upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                    }
                    intakeState = IntakeState.POST;
                }

                if(toOrigin.toTrue()){
                    upper.setSpinWristIntake();
                    upper.setArmPosition(0);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    sequence = Sequence.RUN;
                }
            }

            if(sequence == Sequence.INTAKE_SPECIMEN){
                if(grab.toTrue()){
                    upper.switchClawState();
                }

                if(toReleaseHighChamber.toTrue()){
                    upper.setWristReleaseChamber();
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER_TELEOP);
                    delay(300);
                    upper.setSpinWristRelease_specimen();
                    upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
                    sequence = Sequence.RELEASE_SPECIMEN;
                }
                if(toOrigin.toTrue()){
                    upper.setSpinWristIntake();
                    upper.setArmPosition(0);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    sequence = Sequence.RUN;
                }
            }

            if(sequence == Sequence.RELEASE_SAMPLE){
                if(grab.toTrue()){
                    upper.switchClawState();
                    upper.setWristPostRelease();
                    sequence = Sequence.RUN;
                    upper.setArmPosition(0);
                    delay(200);
                    upper.setSlidePosition(0);
                }
            }

            if(sequence == Sequence.RELEASE_SPECIMEN){
                if(toPullDownSpecimen.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN_TELEOP);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    // TODO: 确保大臂下来不会撞到 chamber
                    sequence = Sequence.RUN;
                    intakeState = IntakeState.NEAR;
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    upper.setWristPreIntake();
                }
            }


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", current_pos.getX(), current_pos.getY(), Math.toDegrees(current_pos.getHeading()));
            telemetry.addData("Position: ", data);
            telemetry.addData("Sequence: ", sequence);
            telemetry.addData("Intake State: ", intakeState);
            telemetry.addData("Trans coefficient", upper.translation_coefficient());
            telemetry.addData("Heading coefficient", upper.heading_coefficient());
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
