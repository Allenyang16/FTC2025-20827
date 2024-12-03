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
            drive.setGlobalPower(upper.translation_coefficient() * gamepad1.left_stick_y, upper.translation_coefficient() * gamepad1.left_stick_x, upper.heading_coefficient() * gamepad1.right_stick_x);
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

        // TODO: try to remove as much delay as possible
        while (opModeIsActive()){
            Pose2d current_pos = drive.getPoseEstimate();

            if(resetHeading.toTrue()){
                drive.resetHeading();
            }

            if(sequence == Sequence.RUN){

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
                    if(intakeState == IntakeState.NEAR){
                        upper.setWristPreIntake();
                        upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                        upper.setClawOpen();
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
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
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
                    upper.setWristPreIntake();
                    if(intakeState == IntakeState.FAR){
                        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    }else{
                        upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                    }
                    intakeState = IntakeState.POST;
                }

                if(toOrigin.toTrue()){
                    // TODO: Check the speed
                    upper.setSpinWristIntake();
                    upper.setArmPosition(0);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    sequence = Sequence.RELEASE_SAMPLE;
                }

                if(intakeState == IntakeState.SPECIMEN){
                    if(toReleaseHighChamber.toTrue()){
                        upper.setWristReleaseChamber();
                        upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
                        delay(300);
                        upper.setSpinWristRelease_specimen();
                        delay(300);
                        upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH);
                        sequence = Sequence.RELEASE_SPECIMEN;
                    }
                }
            }

            if(sequence == Sequence.RELEASE_SAMPLE){
                if(toHighRelease.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
                    delay(200);
                    upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
                    upper.setWristReleaseBox();
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    upper.setWristPostRelease();
                    sequence = Sequence.RUN;
                    upper.setArmPosition(100);
                    delay(150);
                    upper.setSlidePosition(0);
                    delay(400);
                    // TODO: CHECK THE SPEED
                }
            }

            if(sequence == Sequence.RELEASE_SPECIMEN){
                if(toPullDownSpecimen.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    // TODO: 确保大臂下来不会撞到 chamber
                    sequence = Sequence.RUN;
                    intakeState = IntakeState.NEAR;
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    delay(300);
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
