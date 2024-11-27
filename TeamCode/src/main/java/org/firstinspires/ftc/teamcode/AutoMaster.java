package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@Config
public abstract class AutoMaster extends LinearOpMode {

    public static final int POSITIVE = 1;
    public static final int NEGATIVE = -1;
    public static final int RED = -1;
    public static final int BLUE = 1;

    protected int startSide;
    protected int side_color;

    private NewMecanumDrive drive;
    private SuperStructure upper;
    private Runnable update;

    public static int correcting_time = 500, correcting_time2 = 200;

    Pose2d startPos;
    public static double startPos_x = 15, startPos_y = 62, startPos_heading = -90;

    Pose2d boxPos;
    public static double box_x = 54, box_y = 55, box_heading = -45;

    Pose2d chamberPos;
    public static double chamber_x = 6, chamber_y = 38, chamber_heading = -90;

    Pose2d preChamberPos;
    public static double preChamber_x = 6, preChamber_y = 50;

    Pose2d postChamberPos;
    public static double postChamber_x = 30, postChamber_y = 45;

    // TODO: TEST VALUE of all these poses
    Pose2d intakeSamplePos_1;
    public static double intake_samplePos1_x = 49, intake_samplePos1_y = 39.5, intake_samplePos1_heading = -90; // Degree
    Pose2d intakeSamplePos_2;
    public static double intake_samplePos2_x = 58, intake_samplePos2_y = 39.5, intake_samplePos2_heading = -90;
    Pose2d intakeSamplePos_3;
    public static double intake_samplePos3_x = 60, intake_samplePos3_y = 38, intake_samplePos3_heading = -120;

    Pose2d intake_blueSamplePos_1;
    Pose2d intake_blueSamplePos_2;
    Pose2d intake_blueSamplePos_3;

    Pose2d intakeSpecimenPos;
    public static double intakeSpecimen_x = 40, intakeSpecimen_y = 57.2, intakeSpecimen_heading = -90;
    Pose2d preIntakeSpecimenPos;
    public static double pre_intakeSpecimen_y = 50;
    Pose2d dropSampleToHPPos;
    public static double dropSampleToHP_x = 50;

    public static Pose2d endPos = new Pose2d(12,-54.5, Math.toRadians(90));


    protected void initHardware() throws InterruptedException{
        // TODO: must make sure that these poses are correct
        startPos = new Pose2d(startPos_x * startSide ,startPos_y * side_color,Math.toRadians(startPos_heading * side_color));
        boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading * side_color));
        chamberPos = new Pose2d(chamber_x * startSide, chamber_y * side_color, Math.toRadians(chamber_heading * side_color));
        preChamberPos = new Pose2d(preChamber_x * startSide, preChamber_y * side_color, Math.toRadians(chamber_heading * side_color));
        postChamberPos = new Pose2d(postChamber_x * startSide, postChamber_y * side_color, Math.toRadians(chamber_heading * side_color));

        intakeSamplePos_1 = new Pose2d(intake_samplePos1_x * startSide, intake_samplePos1_y * side_color, Math.toRadians(intake_samplePos1_heading * side_color));
        intakeSamplePos_2 = new Pose2d(intake_samplePos2_x * startSide, intake_samplePos2_y * side_color, Math.toRadians(intake_samplePos2_heading * side_color));
        intakeSamplePos_3 = new Pose2d(intake_samplePos3_x * startSide, intake_samplePos3_y * side_color, Math.toRadians(intake_samplePos3_heading * side_color));

        intakeSpecimenPos = new Pose2d(intakeSpecimen_x * startSide, intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        preIntakeSpecimenPos = new Pose2d(intakeSpecimen_x * startSide, pre_intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        dropSampleToHPPos = new Pose2d(dropSampleToHP_x * startSide, pre_intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));

        intake_blueSamplePos_1 = new Pose2d(-40 * startSide, 40 * side_color, Math.toRadians(-90 * side_color));

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new NewMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.update();
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(10));

        telemetry.addLine("init: superstructure");
        telemetry.update();
        upper = new SuperStructure(this);

        update = ()->{
            drive.update();
            upper.update();
            telemetry.update();
        };

        drive.setUpdateRunnable(update);
        upper.setUpdateRunnable(update);

        upper.initialize();

        telemetry.addLine("init: trajectory");
        telemetry.update();
    }

    protected void toOrigin(){
        upper.setWristIntake();
        upper.setArmPosition(0);
        delay(500);
        upper.setSlidePosition(0);
        delay(500);
    }

    protected void moveToDropFirst_sample(){
        drive.setSimpleMoveTolerance(0.8,0.8,Math.toRadians(3));
        drive.setSimpleMovePower(0.9);

        upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
        drive.moveTo(boxPos,correcting_time);
        // Drop
        dropSample();
    }

    protected void moveToDrop_sample(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
        drive.setSimpleMovePower(0.9);

        upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
        drive.moveTo(boxPos,correcting_time2);
        // Drop
        dropSample();
    }
    protected void moveToStartPos(){
        drive.moveTo(startPos, 500);
    }

    protected void intakeSample_1(){
        upper.setWristIntake_ParallelToGround();
        upper.setArmPosition(SuperStructure.ARM_INTAKE);

        drive.moveTo(intakeSamplePos_1, correcting_time2);
        upper.setWristIntake();
        delay(400);
        upper.setClawGrab();
//        delay(200);
    }
    protected void intakeSample_2(){
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSamplePos_2,correcting_time2);
        upper.setClawGrab();
    }
    protected void intakeSample_3(){
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSamplePos_3, correcting_time2);
        upper.setClawGrab();
    }

    protected void dropSample(){
        upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
        delay(1200);
        upper.setWristReleaseBox();
        delay(200);
        upper.setClawOpen();
        delay(200);
    }

    protected void dropSampleToHP(){
        upper.setWristIntakeSpecimen();
        upper.setSpinWristIntake_specimen();
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        drive.moveTo(dropSampleToHPPos,200);
        upper.setClawOpen();
    }
    protected void intakeSpecimen(){
        upper.setWristIntakeSpecimen();
        delay(100);
        upper.setSpinWristIntake_specimen();
        upper.setClawOpen();
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        drive.moveTo(preIntakeSpecimenPos,0);

        drive.setSimpleMovePower(0.2);
        drive.moveTo(intakeSpecimenPos,500);
        delay(500);
        upper.setClawGrab();
    }

    protected void releaseSpecimen(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(2));
        drive.setSimpleMovePower(0.95);

        upper.setWristIntake();
        upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
        upper.setSpinWristRelease_specimen();
        upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH);
        drive.moveTo(preChamberPos,0);

        drive.moveTo(chamberPos,500);

        upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
        delay(500);
        upper.setClawOpen();
    }
    protected void dropSpecimen_toOrigin(){
        upper.setSlidePosition(0);
        drive.moveTo(postChamberPos,0);
    }

    protected void dropSpecimen_toIntakeSpecimen() {

    }

    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}
