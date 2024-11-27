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

        XCYBoolean grab = new XCYBoolean(()-> gamepad2.right_bumper);
        // TODO: test the logic
        XCYBoolean resetHeading = new XCYBoolean(()-> gamepad1.x);
        XCYBoolean toOrigin = new XCYBoolean(()-> (intakeState == IntakeState.POST || intakeState == IntakeState.SPECIMEN) && gamepad1.left_stick_button);
        XCYBoolean toPostIntake = new XCYBoolean(()-> (intakeState != IntakeState.SPECIMEN) && gamepad1.right_stick_button);

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad2.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad2.a);
        XCYBoolean intakeSpecimen = new XCYBoolean(()->gamepad2.b);
        XCYBoolean toHighRelease = new XCYBoolean(()-> gamepad2.dpad_up);
        XCYBoolean downWrist = new XCYBoolean(()-> gamepad2.left_bumper);
        XCYBoolean spinWristClockwise = new XCYBoolean(()-> gamepad2.right_trigger > 0);
        XCYBoolean spinWristCounterClockwise = new XCYBoolean(()-> gamepad2.left_trigger > 0);
        XCYBoolean toReleaseHighChamber = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_up);
        XCYBoolean toPullDownSpecimen = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_down);


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
                //upper.setArmPosition(SuperStructure.ARM_INTAKE);

                if(downWrist.toTrue()){
                    heading_coefficient = 0.1;
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
                        upper.setWristIntake_ParallelToGround();
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
                    upper.setWristIntake_ParallelToGround();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setClawOpen();
                    intakeState = IntakeState.NEAR;
                }

                if(intakeSpecimen.toTrue()){
                    upper.setWristIntakeSpecimen();
                    upper.setSpinWristIntake_specimen();
                    upper.setClawGrab();
                    translation_coefficient = 0.3;
                    heading_coefficient = 0.15;

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
                    translation_coefficient = 1.0;
                    heading_coefficient = 0.6;
                    upper.setSpinWristRelease_specimen();
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    sequence = Sequence.RELEASE_SAMPLE;
                }

                if(intakeState == IntakeState.SPECIMEN){
                    if(toReleaseHighChamber.toTrue()){
                        upper.setWristIntake();
                        upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
                        upper.setSpinWristRelease_specimen();
                        delay(1000);
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
                    delay(200);
                    heading_coefficient = 0.2;
                    translation_coefficient = 0.4;
                    delay(500);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    upper.setWristIntake();
                    sequence = Sequence.RUN;
                    upper.setArmPosition(100);
                    delay(500);
                    upper.setSlidePosition(0);
                    delay(150);
                    heading_coefficient = 0.5;
                    translation_coefficient = 1.0;
                    delay(500);
                }
            }

            if(sequence == Sequence.RELEASE_SPECIMEN){
                if(toPullDownSpecimen.toTrue()){
                    heading_coefficient = 0.5;
                    translation_coefficient = 1.0;
                    upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
                    delay(500);
                    upper.setClawOpen();
                    delay(800);
                    sequence = Sequence.RUN;
                    intakeState = IntakeState.NEAR;
                }
//                if(grab.toTrue()){
//                    upper.switchClawState();
//                    delay(500);
//                    // TODO: 确保大臂下来不会撞到 chamber
//                    sequence = Sequence.RUN;
//                    intakeState = IntakeState.NEAR;
//                }
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
