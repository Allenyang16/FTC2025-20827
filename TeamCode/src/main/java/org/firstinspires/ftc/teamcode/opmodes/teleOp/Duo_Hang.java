package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AutoMaster;
import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

import java.util.Locale;

@TeleOp (name = "TeleOp_DuoHang")
public class Duo_Hang extends LinearOpMode {
    Runnable update;
    int coe = 1;
    enum Sequence{
        RUN, INTAKE_SAMPLE, INTAKE_SPECIMEN, RELEASE_SAMPLE,RELEASE_SPECIMEN, HANG
    }
    enum IntakeState{
        INTAKE_SAMPLE, POST_FAR,POST_NEAR,POST,SPECIMEN
    }
    enum HangState{
        HANG,HANG_OPENLOOP
    }
<<<<<<< HEAD
    private Sequence sequence;
    private IntakeState intakeState;
    private HangState hangState;
=======


    private Duo.Sequence sequence;
    private Duo.IntakeState intakeState;
    private Duo_Hang.HangState hangState;
<<<<<<< Updated upstream
=======
>>>>>>> fafaf4ecda4afff267d9c8604ff5085ebe50ee54
>>>>>>> Stashed changes

    private DcMotor mArmLeft = null;
    private DcMotor mArmRight = null;
    private DcMotor mSlideLeft = null;
    private DcMotor mSlideRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
        mArmLeft = hardwareMap.get(DcMotor.class, "armLeft");
        mArmRight = hardwareMap.get(DcMotor.class, "armRight");
        mSlideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        mSlideRight = hardwareMap.get(DcMotor.class, "slideRight");

        mSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mArmRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mArmLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        mArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SuperStructure upper = new SuperStructure(this);
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        sequence = Sequence.RUN;
        update = ()->{
            drive.update();
            upper.update();
            XCYBoolean.bulkRead();
            telemetry.update();
            drive.setGlobalPower(coe * upper.translation_coefficient() * gamepad1.left_stick_y, coe*upper.translation_coefficient() * gamepad1.left_stick_x, coe*upper.heading_coefficient() * (gamepad1.right_stick_x));
        };
        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

<<<<<<< HEAD
        XCYBoolean resetHeading = new XCYBoolean(()-> gamepad1.x);
        XCYBoolean toOrigin = new XCYBoolean(()-> (gamepad1.left_stick_button));
        XCYBoolean toPostIntake = new XCYBoolean(()-> (intakeState != IntakeState.SPECIMEN) && gamepad1.right_stick_button);
        XCYBoolean toHang = new XCYBoolean(() -> gamepad1.dpad_left && sequence == Sequence.RUN);
        XCYBoolean hang = new XCYBoolean(() -> gamepad1.dpad_right && sequence == Sequence.HANG);
        XCYBoolean toHang_high = new XCYBoolean(() -> gamepad1.dpad_left && sequence == Sequence.HANG);
        XCYBoolean resetSlide = new XCYBoolean(()-> sequence == Sequence.RUN && gamepad1.right_stick_button);
=======
        XCYBoolean resetHeading = new XCYBoolean(() -> gamepad1.x);
        XCYBoolean toOrigin = new XCYBoolean(() -> (gamepad1.left_stick_button));
        XCYBoolean toPostIntake = new XCYBoolean(() -> (gamepad1.right_stick_button));
        XCYBoolean toHang = new XCYBoolean(() -> gamepad1.dpad_left && sequence == Duo.Sequence.RUN);
        XCYBoolean hang = new XCYBoolean(() -> gamepad1.dpad_right && sequence == Duo.Sequence.HANG);
        XCYBoolean toHang_high = new XCYBoolean(() -> gamepad1.dpad_left && sequence == Duo.Sequence.HANG);
<<<<<<< Updated upstream
=======
>>>>>>> fafaf4ecda4afff267d9c8604ff5085ebe50ee54
>>>>>>> Stashed changes

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad2.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad2.a);
        XCYBoolean toIntakeSpecimen = new XCYBoolean(()->gamepad2.b);
        XCYBoolean toHighRelease_sample = new XCYBoolean(()-> gamepad2.dpad_up && sequence == Sequence.RUN);
        XCYBoolean changeWrist = new XCYBoolean(()-> gamepad2.left_bumper && sequence == Sequence.INTAKE_SAMPLE);

        XCYBoolean upWrist = new XCYBoolean(()-> sequence == Sequence.INTAKE_SPECIMEN && gamepad2.left_bumper);
        XCYBoolean grab = new XCYBoolean(()-> gamepad2.right_bumper);

        XCYBoolean spinWristClockwise = new XCYBoolean(()-> gamepad2.right_trigger > 0 && sequence == Sequence.INTAKE_SAMPLE);
        XCYBoolean spinWristCounterClockwise = new XCYBoolean(()-> gamepad2.left_trigger > 0);
        XCYBoolean toReleaseHighChamber = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_up);
        XCYBoolean toPullDownSpecimen = new XCYBoolean(()-> intakeState == IntakeState.SPECIMEN && gamepad2.dpad_down);

        upper.initialize();
        drive.setPoseEstimate(AutoMaster.endPos);
        drive.setYawHeading(AutoMaster.yawOffset);

<<<<<<< HEAD
        intakeState = IntakeState.POST_NEAR;
        hangState = HangState.HANG;
=======
        intakeState = Duo.IntakeState.POST_NEAR;
        hangState = Duo_Hang.HangState.HANG;
<<<<<<< Updated upstream
=======
>>>>>>> fafaf4ecda4afff267d9c8604ff5085ebe50ee54
>>>>>>> Stashed changes
        waitForStart();

        while (opModeIsActive()){
            Pose2d current_pos = drive.getPoseEstimate();

            if(resetHeading.toTrue()){
                drive.resetHeading();
            }
<<<<<<< Updated upstream
=======
<<<<<<< HEAD

            if (sequence == Sequence.HANG) {

                if (toOrigin.toTrue()) {
                    upper.setSlidePosition_horizontal(0);
                    delay(300);
                    upper.setArmPosition(0);
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();
                    sequence = Sequence.RUN;
                }
                if (hang.toTrue()) {
                    upper.setSlidePosition_hang(SuperStructure.SLIDE_HANG_LOW_DOWN);
                    delay(1800);
                    upper.setArmPosition(-50);
                }
                if (toHang_high.toTrue()) {
                    upper.setArmPosition(-80);
                    delay(100);
                    upper.hang_setSlide(SuperStructure.SLIDE_HANG_HIGH_UP);
                    delay(200);
                    upper.setArmPosition(120);
                    sequence = Sequence.HANG;
                    hangState = HangState.HANG_OPENLOOP;

                }
                if (sequence == Sequence.HANG && hangState == HangState.HANG_OPENLOOP) {
                    if (gamepad1.dpad_up) {
                        coe = 0;
                        while(true) {
                            upper.slidePower = -gamepad1.left_stick_y;
                            upper.armPower = gamepad1.right_stick_y;
                            upper.setUpperHang();
                            if (toOrigin.toTrue()) {
                                coe = 1;
                                break;
                            }
=======
>>>>>>> Stashed changes
            if (hang.toTrue()) {
                upper.setSlidePosition_hang(SuperStructure.SLIDE_HANG_LOW_DOWN);
                delay(1800);
                upper.setArmPosition(-50);
            }
            if (toHang_high.toTrue()) {
                upper.setArmPosition(-80);
                delay(100);
                upper.hang_setSlide(SuperStructure.SLIDE_HANG_HIGH_UP);
                delay(200);
                upper.setArmPosition(50);
                sequence = Duo.Sequence.HANG;
                hangState = HangState.HANG_OPENLOOP;
            }
            if (sequence == Duo.Sequence.HANG && hangState ==HangState.HANG_OPENLOOP) {
                if (gamepad1.dpad_up) {
                    coe = 0;
                    while(true) {
                        upper.slidePower = -gamepad1.left_stick_y;
                        upper.armPower = gamepad1.right_stick_y;
                        upper.setUpperHang();
                        if (toOrigin.toTrue()) {
                            coe = 1;
                            break;
>>>>>>> fafaf4ecda4afff267d9c8604ff5085ebe50ee54
                        }
                    }

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
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
                    sequence = Sequence.HANG;
//                    upper.setArmPosition(SuperStructure.ARM_HANG_LOW);
                }
                if(intakeFar.toTrue()){
=======
>>>>>>> Stashed changes
                    sequence = Duo.Sequence.HANG;
                }
                if (intakeFar.toTrue()) {
>>>>>>> fafaf4ecda4afff267d9c8604ff5085ebe50ee54
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
                    upper.setWristPreIntake();
                    upper.setWristIntake();
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
                    if (intakeState == IntakeState.POST_NEAR || intakeState == IntakeState.POST_FAR) {
                        if (upper.clawState == SuperStructure.ClawState.OPEN){
                            upper.setArmPosition(SuperStructure.ARM_INTAKE);
                            delay(120);
                            upper.switchClawState();
                            delay(80);
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
                if(changeWrist.toTrue()){
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
                    upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
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
                    upper.setWristPreIntake();

                    intakeState = IntakeState.POST_NEAR;
                }

                if (toIntakeSpecimen.toTrue()) {
                    if (intakeState == IntakeState.INTAKE_SAMPLE){
                        upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
                        delay(100);
                    }
                    upper.setWristIntakeSpecimenGround();
                    upper.setSpinWristIntake_specimen();
                    intakeState = IntakeState.SPECIMEN;
                    sequence = Sequence.INTAKE_SPECIMEN;
                }

                if(toPostIntake.toTrue()){
                    upper.setWristPreIntake();
                    upper.setSpinWristIntake();
<<<<<<< Updated upstream
                    upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
                    intakeState = Duo.IntakeState.POST_NEAR;
=======
<<<<<<< HEAD
                    if(intakeState == IntakeState.POST_FAR){
                        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
                        intakeState = IntakeState.POST_NEAR;
                    }else{
                        upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                        intakeState = IntakeState.POST;
                    }
                    upper.setWristPreIntake();
=======
                    upper.setArmPosition(SuperStructure.ARM_POST_INTAKE);
                    upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
                    intakeState = Duo.IntakeState.POST_NEAR;
>>>>>>> fafaf4ecda4afff267d9c8604ff5085ebe50ee54
>>>>>>> Stashed changes
                }

                if(toOrigin.toTrue() && intakeState == IntakeState.POST_NEAR){
                    upper.setSpinWristIntake();
                    upper.setArmPosition(0);
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
                    sequence = Sequence.RUN;
=======
>>>>>>> fafaf4ecda4afff267d9c8604ff5085ebe50ee54
>>>>>>> Stashed changes
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
                    upper.setSlidePosition_verticle(0);
                    delay(50);

                    upper.setWristReleaseChamber();
                    upper.setSpinWristIntake();
                    upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER_TELEOP);
                    delay(350);
                    upper.setSlidePosition_verticle(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
                    sequence = Sequence.RELEASE_SPECIMEN;
                }

                if(toOrigin.toTrue() ){
                    upper.setSlidePosition_horizontal(0);
                    delay(300);
                    upper.setArmPosition(0);
                    upper.setWristPreIntake();
                    upper.setWristReleaseChamber();
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
