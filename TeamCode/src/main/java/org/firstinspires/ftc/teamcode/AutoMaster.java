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
    public static double box_x = 57, box_y = 57, box_heading = 45; // or 135 in blue

    Pose2d chamberPos;
    public static double chamber_x = 6, chamber_y = 39, chamber_heading = -90;
    Pose2d chamberPos2;
    public static double chamber2_x = 8;
    Pose2d chamberPos_delta;
    public static double chamber_delta_x = 1;
    Pose2d preChamberPos;
    public static double preChamber_x = 8, preChamber_y = 48;

    Pose2d postChamberPos;
    public static double postChamber_x = 33, postChamber_y = 45;


    Pose2d intakeSamplePos_1;
    public static double intake_samplePos1_x = 48, intake_samplePos1_y = 40.5, intake_samplePos1_heading = -90; // Degree
    Pose2d intakeSamplePos_2;
    public static double intake_samplePos2_x = 58, intake_samplePos2_y = 39.5, intake_samplePos2_heading = -90;
    Pose2d intakeSamplePos_3;
    public static double intake_samplePos3_x = 60, intake_samplePos3_y = 37, intake_samplePos3_heading = -120;
    public static double sample3_positive_heading = -60;

    Pose2d intake_blueSamplePos_1;
    Pose2d intake_blueSamplePos_2;
    Pose2d intake_blueSamplePos_3;

    Pose2d pushSamplePos_1;
    Pose2d pushSamplePos_2;
    Pose2d pushSamplePos_3;
    Pose2d pushSamplePos_delta;
    Pose2d pushSamplePos_midpoint;
    public static double pushSample1_x = 45, pushSample2_x = 58, pushSample3_x = 63, pushMidpoint_x = 33;
    public static double pushSample_y = 15, pushSample_heading = -90, pushSample_delta_y = 32, pushMidpoint_y = 15;

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
        if(side_color == RED){
            boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading));
        }else{
            boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(-135));
        }

        chamberPos = new Pose2d(chamber_x * startSide, chamber_y * side_color, Math.toRadians(chamber_heading * side_color));
        chamberPos2 = new Pose2d(chamber2_x * startSide, chamber_y * side_color, Math.toRadians(chamber_heading * side_color));
        chamberPos_delta = new Pose2d(chamber_delta_x * startSide, 0, 0);

        preChamberPos = new Pose2d(preChamber_x * startSide, preChamber_y * side_color, Math.toRadians(chamber_heading * side_color));
        postChamberPos = new Pose2d(postChamber_x * startSide, postChamber_y * side_color, Math.toRadians(chamber_heading * side_color));

        intakeSamplePos_1 = new Pose2d(intake_samplePos1_x * startSide, intake_samplePos1_y * side_color, Math.toRadians(intake_samplePos1_heading * side_color));
        intakeSamplePos_2 = new Pose2d(intake_samplePos2_x * startSide, intake_samplePos2_y * side_color, Math.toRadians(intake_samplePos2_heading * side_color));
        if(startSide == POSITIVE){
            intakeSamplePos_3 = new Pose2d(intake_samplePos3_x * startSide, intake_samplePos3_y * side_color, Math.toRadians(sample3_positive_heading * side_color));
        }else{
            intakeSamplePos_3 = new Pose2d(intake_samplePos3_x * startSide, intake_samplePos3_y * side_color, Math.toRadians(intake_samplePos3_heading * side_color));
        }

        intakeSpecimenPos = new Pose2d(intakeSpecimen_x * startSide, intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        preIntakeSpecimenPos = new Pose2d(intakeSpecimen_x * startSide, pre_intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        dropSampleToHPPos = new Pose2d(dropSampleToHP_x * startSide, pre_intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));

        pushSamplePos_1 = new Pose2d(pushSample1_x * startSide, pushSample_y * side_color, Math.toRadians(pushSample_heading * side_color) );
        pushSamplePos_2 = new Pose2d(pushSample2_x * startSide, pushSample_y * side_color, Math.toRadians(pushSample_heading  * side_color) );
        pushSamplePos_3 = new Pose2d(pushSample3_x * startSide, pushSample_y * side_color, Math.toRadians(pushSample_heading * side_color) );
        pushSamplePos_delta = new Pose2d(0, pushSample_delta_y * side_color, 0);

        pushSamplePos_midpoint = new Pose2d(pushMidpoint_x * startSide, pushMidpoint_y * side_color, Math.toRadians(pushSample_heading * side_color));
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
        upper.setSpinWristIntake();
        drive.moveTo(boxPos,correcting_time2);
        // Drop
        dropSample();
    }
    protected void moveToStartPos(){
        drive.moveTo(startPos, 500);
    }

    protected void intakeSample_1(){
        upper.setWristPreIntake();
        upper.setArmPosition(SuperStructure.ARM_INTAKE);

        drive.moveTo(intakeSamplePos_1, correcting_time2);
        upper.setWristIntake();
        delay(200);
        upper.setClawGrab();
//        delay(200);
    }
    protected void intakeSample_2(){
        upper.setWristPreIntake();
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSamplePos_2,correcting_time2);
        upper.setWristIntake();
        delay(200);
        upper.setClawGrab();
    }
    protected void intakeSample_3(){
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        upper.setSpinWristIntake_spinClockwise();
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
        drive.moveTo(dropSampleToHPPos,400);
        upper.setClawOpen();
    }
    protected void intakeSpecimen(){
        upper.setWristIntakeSpecimen();
        upper.setSpinWristIntake_specimen();
        upper.setClawOpen();
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        drive.moveTo(preIntakeSpecimenPos,0);

        drive.setSimpleMovePower(0.5);
        drive.moveTo(intakeSpecimenPos,500);
        upper.setClawGrab();
    }

    protected void releaseSpecimen(int count){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(2));
        drive.setSimpleMovePower(0.95);

        upper.setWristIntake();

        upper.setSpinWristRelease_specimen();
        upper.setArmPosition(0);
        upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);

        if(count != 1){
            drive.moveTo(preChamberPos,0);
            upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
            upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH);

            drive.setSimpleMovePower(0.6);
            if (count == 2) {
                drive.moveTo(chamberPos.plus(chamberPos_delta),335);
            } else if (count == 3) {
                drive.moveTo(chamberPos.plus(chamberPos_delta).plus(chamberPos_delta),335);
            } else if (count == 4) {
                drive.moveTo(chamberPos.plus(chamberPos_delta).plus(chamberPos_delta).plus(chamberPos_delta),335);
            }
        } else {
            upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
            upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH);

            drive.setSimpleMovePower(0.6);
            drive.moveTo(chamberPos,480);
        }

        upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
        delay(300);
        upper.setClawOpen();
    }

    protected void dropSpecimen_toOrigin(){
        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
        drive.moveTo(postChamberPos,0);
    }
    public void park(){
        upper.setArmPosition(0);
        drive.moveTo(intakeSpecimenPos,500);
    }

    public void pushSample(){
        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(3,3,1);
        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
        upper.setArmPosition(0);
        // Move to the first pre push pos
        drive.moveTo(postChamberPos,0);
        drive.moveTo(pushSamplePos_midpoint,0);

        drive.moveTo(pushSamplePos_1,0);
        drive.moveTo(pushSamplePos_1.plus(pushSamplePos_delta),0);
        drive.moveTo(pushSamplePos_1,0);

        drive.moveTo(pushSamplePos_2,0);
        drive.moveTo(pushSamplePos_2.plus(pushSamplePos_delta),0);
    }

    protected void dropSpecimen_toIntakeSpecimen() {
        drive.setSimpleMovePower(0.95);

        upper.setWristIntakeSpecimen();
        upper.setSpinWristIntake_specimen();
        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        drive.moveTo(preIntakeSpecimenPos,0);

        drive.setSimpleMovePower(0.5);
        drive.moveTo(intakeSpecimenPos,500);
        upper.setClawGrab();
    }

    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}