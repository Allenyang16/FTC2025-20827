package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

import java.util.Locale;

@TeleOp (name = "TeleOp_Duo")
public class Duo extends LinearOpMode {
    Runnable update;
    enum Sequence{
        RUN, INTAKE_SAMPLE, INTAKE_SPECIMEN, RELEASE_SAMPLE,RELEASE_SPECIMEN,HANG
    }
    enum IntakeState{
        INTAKE_SAMPLE, POST_FAR, POST_NEAR, POST, SPECIMEN
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
            if (sequence == Duo.Sequence.HANG) {
                drive.setGlobalPower(0, 0, 0);
            }else {
                drive.setGlobalPower(upper.translation_coefficient() * gamepad1.left_stick_y, upper.translation_coefficient() * gamepad1.left_stick_x, upper.heading_coefficient() * gamepad1.right_stick_x);
            }
        };
        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        XCYBoolean resetHeading = new XCYBoolean(()-> gamepad1.x);
        XCYBoolean toOrigin = new XCYBoolean(()-> (gamepad1.left_stick_button));
        XCYBoolean toPostIntake = new XCYBoolean(()-> (intakeState != IntakeState.SPECIMEN) && gamepad1.right_stick_button);
        XCYBoolean toHang = new XCYBoolean(() -> gamepad1.dpad_left && sequence == Sequence.RUN);
        XCYBoolean hang = new XCYBoolean(() -> gamepad1.dpad_right && sequence == Sequence.HANG);
        XCYBoolean toHang_high = new XCYBoolean(() -> gamepad1.dpad_left && sequence == Sequence.HANG);

        XCYBoolean hang_high = new XCYBoolean(() -> gamepad2.dpad_right && sequence == Sequence.HANG);
        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad2.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad2.a);
        XCYBoolean toIntakeSpecimen = new XCYBoolean(()->gamepad2.b);
        XCYBoolean toHighRelease_sample = new XCYBoolean(()-> gamepad2.dpad_up && sequence == Sequence.RUN);
        XCYBoolean changeWrist = new XCYBoolean(()-> gamepad2 .left_bumper && sequence == Sequence.INTAKE_SAMPLE);
        XCYBoolean upWrist = new XCYBoolean(()-> sequence == Sequence.INTAKE_SPECIMEN && gamepad2.left_bumper);
        XCYBoolean grab = new XCYBoolean(()-> gamepad2.right_bumper);

        XCYBoolean spinWristClockwise = new XCYBoolean(()-> gamepad2.right_trigger > 0 && sequence == Sequence.INTAKE_SAMPLE);
        XCYBoolean spinWristCounterClockwise = new XCYBoolean(()-> gamepad2.left_trigger >0 && sequence == Sequence.INTAKE_SAMPLE);
        XCYBoolean toReleaseHighChamber = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_up);
        XCYBoolean toPullDownSpecimen = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_down);
        XCYBoolean resetSlide = new XCYBoolean(()-> sequence == Sequence.RUN && gamepad2.right_stick_button);

        upper.initialize();
        drive.setPoseEstimate(AutoMaster.endPos);
        drive.setYawHeading(AutoMaster.yawOffset);

        intakeState = IntakeState.POST_NEAR;
        waitForStart();

        while (opModeIsActive()){
            Pose2d current_pos = drive.getPoseEstimate();

            if(resetHeading.toTrue()){
                drive.resetHeading();
            }

            if (sequence == Sequence.HANG) {
                if (hang.toTrue()) {
                    upper.setSlidePosition_hang(SuperStructure.SLIDE_HANG_LOW_DOWN);
                    delay(1800);
                    upper.setArmPosition(100);
                }

                if (toOrigin.toTrue()) {
                    upper.setSlidePosition_horizontal(0);
                    delay(300);
                    upper.setArmPosition(0);
                    upper.setWristPreIntake();
                    upper.setWristReleaseChamber();
                    sequence = Sequence.RUN;
                }

                if (toHang_high.toTrue()) {
                    upper.setArmPosition(80);
                    delay(10);
                    upper.hang_setSlide(SuperStructure.SLIDE_HANG_HIGH_UP);
                    delay(500);
                    upper.disableArmMotor();
                    upper.hang_setSlide(SuperStructure.SLIDE_HANG_HIGH_DOWN); //
                    upper.enableArmMotor();
                    upper.setArmPosition(SuperStructure.ARM_HANG_HIGH);
                    delay(500);
                    upper.hang_setSlide(0);
                }

            }

            if(sequence == Sequence.RUN){
                if(resetSlide.toTrue()){
                    upper.setSlidePosition(-50);
                    delay(300);
                    upper.resetSlide();
                    upper.setSlidePosition(0);
                }
                if (toHang.toTrue()) {
                    upper.setArmPosition(SuperStructure.ARM_HANG_LOW);
                    upper.hang_setSlide(SuperStructure.SLIDE_HANG_LOW_UP);
                    delay(500);
                    sequence = Sequence.HANG;
                }
                if(intakeFar.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();
                    delay(200);
                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
                    intakeState = IntakeState.POST_FAR;
                    sequence = Sequence.INTAKE_SAMPLE;
                }

                if(intakeNear.toTrue()){
                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
                    upper.setSpinWristIntake();
                    upper.setWristPreIntake();
                    upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                    intakeState = IntakeState.POST_NEAR;
                    sequence = Sequence.INTAKE_SAMPLE;
                }

                if (toIntakeSpecimen.toTrue()) {
                    upper.setSpinWristIntake_specimen();
                    upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                    delay(300);
                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
                    upper.setWristIntakeSpecimenGround();
                    upper.setClawOpen();

                    intakeState = IntakeState.SPECIMEN;
                    sequence = Sequence.INTAKE_SPECIMEN;
                }


                if(toHighRelease_sample.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
//                    delay(100);
                    upper.setSlidePosition_verticle(SuperStructure.SLIDE_BOX_HIGH);
                    upper.setWristPreIntake();
                    upper.setSpinWristReleaseBox();
                    sequence = Sequence.RELEASE_SAMPLE;
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                }
            }

            if(sequence == Sequence.INTAKE_SAMPLE){
                if(grab.toTrue()){
                    if (intakeState == Duo.IntakeState.POST_NEAR || intakeState == Duo.IntakeState.POST_FAR) {
                        if (upper.clawState == SuperStructure.ClawState.OPEN){
                            upper.setArmPosition(SuperStructure.ARM_INTAKE);
                            delay(100);
                            upper.switchClawState();
                            delay(100);
                            upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                        }
                        else{
                            upper.switchClawState();
                        }
                    }
                    else{
                        upper.switchClawState();
                    }

                }
                if(changeWrist.toTrue()) {
                    upper.switchWristIntakeState();
                    upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                }
                if(spinWristClockwise.toTrue()){
                    upper.setSpinWristIntake_spinClockwise();
                }
                if(spinWristCounterClockwise.toTrue()){
                    upper.setSpinWristIntake_spinCounterClockwise();
                }

                if(intakeFar.toTrue()) {

                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
                    upper.setSpinWristIntake();
                    delay(200);
                    upper.setClawOpen();

                    intakeState = IntakeState.POST_FAR;
                }
                if(intakeNear.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
                    upper.setSpinWristIntake();

                    intakeState = IntakeState.POST_NEAR;
                }

                if (toIntakeSpecimen.toTrue()) {
                    upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                    upper.setWristIntakeSpecimenGround();
                    upper.setSpinWristIntake_specimen();
                    intakeState = IntakeState.SPECIMEN;
                    sequence = Sequence.INTAKE_SPECIMEN;
                }

                if(toPostIntake.toTrue()){
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();

                        upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
                        upper.setWristPreIntake();
                        intakeState = IntakeState.POST_NEAR;

                }

                if(toOrigin.toTrue() && intakeState == IntakeState.POST_NEAR){
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();
                    upper.setArmPosition(0);
                    sequence = Sequence.RUN;
                }
            }

            if(sequence == Sequence.INTAKE_SPECIMEN){
                if(grab.toTrue()){
                    upper.setArmPosition(SuperStructure.ARM_INTAKE);
                    upper.switchClawState();
                }

                if(upWrist.toTrue()){
                    upper.setWristIntakeSpecimenGround();
                    upper.setSpinWristIntake_specimen();
                }
                if(intakeFar.toTrue()){
                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
                    upper.setSpinWristIntake_specimen();
                }
                if(intakeNear.toTrue()){
                    upper.setSlidePosition_horizontal(0);
                    upper.setSpinWristIntake_specimen();
                }

                if(toReleaseHighChamber.toTrue()){
                    upper.setSlidePosition_horizontal(0);
                    delay(100);
                    upper.setWristReleaseChamber();
                    upper.setSpinWristIntake();
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER_TELEOP);
                    delay(350);
                    upper.setSlidePosition_verticle(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
                    sequence = Sequence.RELEASE_SPECIMEN;
                }

                if(toOrigin.toTrue() ){
                    upper.setSlidePosition_horizontal(0);
                    delay(250);
                    upper.setArmPosition(0);
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();
                    sequence = Sequence.RUN;
                }
            }

            if(sequence == Sequence.RELEASE_SAMPLE){
                if(grab.toTrue()){
                    upper.setWristReleaseBox();
                    delay(10);
                    upper.switchClawState();
                    upper.setWristPostRelease();
                    sequence = Sequence.RUN;
                    upper.setArmPosition(0);
                    delay(200);
                    upper.setSlidePosition_verticle(0);
                }
            }

            if(sequence == Sequence.RELEASE_SPECIMEN){
                if(toReleaseHighChamber.toTrue()){
                    upper.setSlidePosition_verticle(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
                }

                if(toPullDownSpecimen.toTrue()){
                    upper.setSlidePosition_verticle(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN_TELEOP);
                }

                if(grab.toTrue()){
                    upper.switchClawState();
                    sequence = Sequence.RUN;
                    intakeState = IntakeState.POST_NEAR;
                    upper.setSlidePosition_verticle(SuperStructure.SLIDE_MIN);
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();
                    upper.setArmPosition(0);
                    sequence = Sequence.RUN;

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
