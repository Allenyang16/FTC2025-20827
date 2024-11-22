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
            drive.setGlobalPower(gamepad1.left_stick_y, gamepad1.left_stick_x, heading_coefficient * gamepad1.right_stick_x);
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
                upper.setArmPosition(SuperStructure.ARM_INTAKE);


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
                    heading_coefficient = 0.2;
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
                    heading_coefficient = 0.2;
                    upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                    upper.setSpinWristIntake();
                    upper.setWristIntake_ParallelToGround();
                    upper.setSlideState(SuperStructure.SlideState.HORIZONTAL);

                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.setClawOpen();
                    // TODO:试一下能不能把delay给去掉
                    delay(500);
                    intakeState = IntakeState.NEAR;
                }

                if(intakeSpecimen.toTrue()){
                    // TODO: decide whether to change this with spinRelease
                    upper.setWristIntakeSpecimen();
                    upper.setSpinWristIntake();
                    upper.setClawOpen();
                    upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);

                    intakeState = IntakeState.SPECIMEN;
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                }

                if(toPostIntake.toTrue()){
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
                    heading_coefficient = 0.5;
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
                    upper.setSlideState(SuperStructure.SlideState.VERTICAL);
                    if(intakeState == IntakeState.SPECIMEN){
                        sequence = Sequence.RELEASE_SPECIMEN;
                    }else{
                        sequence = Sequence.RELEASE_SAMPLE;
                    }
                }
            }

            if(sequence == Sequence.RELEASE_SAMPLE){
                if(toHighRelease.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
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
            }

            if(sequence == Sequence.RELEASE_SPECIMEN){
                if(toReleaseHighChamber.toTrue()){
                    upper.setWristIntake();
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
                    upper.setSpinWristRelease_specimen();
                    upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH);
                }
                if(toPullDownSpecimen.toTrue()){
                    upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    delay(500);
                    // TODO: 确保大臂下来不会撞到 chamber
                    sequence = Sequence.RUN;
                }

            }


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", current_pos.getX(), current_pos.getY(), Math.toDegrees(current_pos.getHeading()));
            telemetry.addData("Position: ", data);
            telemetry.addData("Sequence: ", sequence);
            telemetry.addData("Intake State: ", intakeState);

            // TODO: CHECK whether this will work
            //drive.setGlobalPower(gamepad1.left_stick_y, gamepad1.left_stick_x, headingPower_coefficient * gamepad1.right_stick_x);

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
