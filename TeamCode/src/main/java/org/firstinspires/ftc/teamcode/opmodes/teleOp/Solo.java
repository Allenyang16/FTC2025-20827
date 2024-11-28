package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

import java.util.Locale;

@TeleOp (name = "TeleOp_Solo")
public class Solo extends LinearOpMode {
    Runnable update;
    enum Sequence{
        RUN, RELEASE_SAMPLE,RELEASE_SPECIMEN
    }
    enum IntakeState{
        FAR, NEAR, POST, SPECIMEN
    }
    private Sequence sequence;
    private IntakeState intakeState;
    private static double heading_coefficient = 0.5;
    private static double translation_coefficient = 1.0;

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
            drive.setGlobalPower(translation_coefficient * gamepad1.left_stick_y, translation_coefficient * gamepad1.left_stick_x, heading_coefficient * gamepad1.right_stick_x);
        };
        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad1.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad1.a);
        XCYBoolean intakeSpecimen = new XCYBoolean(()->gamepad1.b);
        XCYBoolean resetHeading = new XCYBoolean(()-> gamepad1.x);

        XCYBoolean toReleaseHighChamber = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad1.dpad_up);
        XCYBoolean toPullDownSpecimen = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad1.dpad_down);

        XCYBoolean grab = new XCYBoolean(()-> gamepad1.right_bumper);
        // TODO: test the logic
        XCYBoolean toOrigin = new XCYBoolean(()-> (intakeState == IntakeState.POST || intakeState == IntakeState.SPECIMEN) && gamepad1.left_stick_button);
        XCYBoolean toPostIntake = new XCYBoolean(()-> (intakeState != IntakeState.SPECIMEN) && gamepad1.right_stick_button);
        XCYBoolean toHighRelease = new XCYBoolean(()-> gamepad1.dpad_up);
        XCYBoolean downWrist = new XCYBoolean(()-> gamepad1.left_bumper);

        XCYBoolean spinWristClockwise = new XCYBoolean(()-> gamepad1.right_trigger > 0);
        XCYBoolean spinWristCounterClockwise = new XCYBoolean(()-> gamepad1.left_trigger > 0);

        upper.initialize();
        drive.setPoseEstimate(AutoMaster.endPos);

        intakeState = IntakeState.NEAR;
        waitForStart();

        while (opModeIsActive()){
            Pose2d current_pos = drive.getPoseEstimate();

            if(resetHeading.toTrue()){
                drive.resetHeading();
            }

            if(sequence == Sequence.RUN){
                if(downWrist.toTrue()){
                    heading_coefficient = 0.15;
                    translation_coefficient = 0.3;
                    upper.switchWristIntakeState();
                }

                if(spinWristClockwise.toTrue()){
                    upper.setSpinWristIntake_spinClockwise();
                }
                if(spinWristCounterClockwise.toTrue()){
                    upper.setSpinWristIntake_spinCounterClockwise();
                }

                if(intakeFar.toTrue()){
                    if(intakeState == IntakeState.NEAR){
                        upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                        intakeState = IntakeState.FAR;
                    }else {
                        upper.setWristPreIntake();
                        upper.setSpinWristIntake();
                        upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                        upper.setArmPosition(SuperStructure.ARM_INTAKE);
                        upper.setClawOpen();
                        upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                        intakeState = IntakeState.FAR;
                    }
                }

                if(intakeNear.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    upper.setSpinWristIntake();
                    upper.setWristPreIntake();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setClawOpen();
                    intakeState = IntakeState.NEAR;
                }

                if(intakeSpecimen.toTrue()){
                    upper.setWristIntakeSpecimen();
                    upper.setSpinWristIntake_specimen();
                    upper.setClawGrab();

                    intakeState = IntakeState.SPECIMEN;
                }

                if(intakeState == IntakeState.SPECIMEN){
                    upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
                }else{
                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                }

                if(toPostIntake.toTrue()){
                    translation_coefficient = 1;
                    heading_coefficient = 0.5;

                    if(intakeState == IntakeState.FAR){
                        upper.setWristPreIntake();
                        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                        delay(500);
                    }else{
                        upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                        upper.setWristPreIntake();
                    }
                    intakeState = IntakeState.POST;
                }

                if(toOrigin.toTrue()){
                    heading_coefficient = 0.6;
                    upper.setSpinWristRelease_specimen();
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    sequence = Sequence.RELEASE_SAMPLE;
                }

                if(intakeState == IntakeState.SPECIMEN){
                    if(toReleaseHighChamber.toTrue()){
                        heading_coefficient = 0.6;
                        translation_coefficient = 1.0;
                        upper.setWristIntake();
                        upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
                        upper.setSpinWristRelease_specimen();
                        delay(500);
                        upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH);
                        sequence = Sequence.RELEASE_SPECIMEN;
                    }
                }
            }

            if(sequence == Sequence.RELEASE_SAMPLE){
                if(toHighRelease.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
                    upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
                    upper.setWristReleaseBox();
                    delay(500);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    upper.setWristIntake();
                    sequence = Sequence.RUN;
                    upper.setArmPosition(0);
                    delay(300);
                    upper.setSlidePosition(0);
                    delay(500);
                }
            }

            if(sequence == Sequence.RELEASE_SPECIMEN){
                if(toPullDownSpecimen.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    delay(200);
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    delay(500);
                    // TODO: 确保大臂下来不会撞到 chamber
                    sequence = Sequence.RUN;
                    intakeState =IntakeState.NEAR;
                }
            }


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", current_pos.getX(), current_pos.getY(), Math.toDegrees(current_pos.getHeading()));
            telemetry.addData("Position: ", data);
            telemetry.addData("Sequence: ", sequence);
            telemetry.addData("Intake State: ", intakeState);
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
