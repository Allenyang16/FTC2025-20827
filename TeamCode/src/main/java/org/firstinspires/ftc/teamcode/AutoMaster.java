package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

    public static int correcting_time = 200, correcting_time2 = 200;
    public static double yawOffset = 0;

    Pose2d startPos;
    public static double startPos_x = 39, startPos_y = 62, startPos_heading = -90;
    public static double startPos_chamber_x = 9, startPos_box_x = 39;

    Pose2d boxPos;
    public static double box_x = 57, box_y = 57, box_heading = 45; // or 135 in blue

    Pose2d chamberPos;
    public static double chamber_x = 6, chamber_y = 35.6, chamber_heading = -90;
    Pose2d chamberPos2;
    public static double chamber2_x = 8;
    Pose2d chamberPos_delta;
    public static double chamber_delta_x = 1.5;
    Pose2d preChamberPos;
    public static double preChamber_x = 6, preChamber_y = 37;

    Pose2d postChamberPos;
    public static double postChamber_x = 35, postChamber_y = 40;

    Pose2d intakeSamplePos_1;
    public static double intake_samplePos1_x = 48, intake_samplePos1_y = 38, intake_samplePos1_heading = -90; // Degree
    Pose2d intakeSamplePos_2;
    public static double intake_samplePos2_x = 58, intake_samplePos2_y = 38, intake_samplePos2_heading = -90;
    Pose2d intakeSamplePos_3;
    public static double intake_samplePos3_x = 57, intake_samplePos3_y = 45, intake_samplePos3_heading = -120;
    public static double sample3_positive_heading = -60;

    public static double intake_redSample1_x = 37, intake_redSample1_y = -47, intake_redSample1_heading = -80;
    public static double intake_redSample2_x = 42, intake_redSample2_y = -47;
    public static double intake_redSample3_x = 60, intake_redSample3_y = 57.2;
    public static double release_redSample_heading_1 = -30;
    public static double release_redSample_heading_2 = -40;
    public static double release_redSample_heading_3 = 90;
    Pose2d intakeRedSample = new Pose2d(intake_redSample3_x,intake_redSample3_y, Math.toRadians(intake_redSample1_heading));
    Pose2d releaseRedSample_1 = new Pose2d(intake_redSample1_x,intake_redSample1_y, Math.toRadians(release_redSample_heading_1));
    Pose2d releaseRedSample_2 = new Pose2d(intake_redSample2_x,intake_redSample2_y, Math.toRadians(release_redSample_heading_2));
    Pose2d releaseRedSample_3 = new Pose2d(intake_redSample3_x,intake_redSample3_y, Math.toRadians(release_redSample_heading_3));

    Pose2d pushSamplePos_1;
    Pose2d pushSamplePos_2;
    Pose2d pushSamplePos_3;
    Pose2d pushSamplePos_delta;
    Pose2d pushSamplePos_midpoint;
    public static double pushSample1_x = 46, pushSample2_x = 58, pushSample3_x = 63, pushMidpoint_x = 35;
    public static double pushSample_y = 15, pushSample_heading = -90, pushSample_delta_y = 32, pushMidpoint_y = 15;

    Pose2d intakeSpecimenPos;
    public static double intakeSpecimen_x = 50, intakeSpecimen_y = 57.2, intakeSpecimen_heading = -90;

    Pose2d intakeSpecimenPos_ground;
    public static double intakeSpecimen_ground_x = 24, intakeSpecimen_ground_y = 45, intakeSpecimenRed_ground_heading = -45;

    Pose2d preIntakeSpecimenPos;
    public static double pre_intakeSpecimen_y = 52;
    Pose2d dropSamplePos;
    public static double dropSampleToHP_x = 50;

    public static Pose2d park_box = new Pose2d(-24,-10,Math.toRadians(90));
    public static double park_box_x = 22, park_box_y = 10, parkRed_box_heading = 0, parkBlue_box_heading = 180;
    public static Pose2d prePark_box;
    public static double prePark_box_x = 45, prePark_box_y = 10;

    public static Pose2d park_chamber = new Pose2d(50, 57.2,Math.toRadians(90));
    public static double park_chamber_x = 56, park_chamber_y = 57.2, park_chamber_heading = -90;

    public static Pose2d endPos = park_box;


    protected void initHardware() throws InterruptedException{
        if(side_color == RED && startSide == POSITIVE){
            startPos_x = startPos_chamber_x;
        } else if (side_color == BLUE && startSide == NEGATIVE) {
            startPos_x = startPos_chamber_x;
        } else{
            startPos_x = startPos_box_x;
        }

        startPos = new Pose2d(startPos_x * startSide ,startPos_y * side_color,Math.toRadians(startPos_heading * side_color));
        if(side_color == RED){
            boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading));
            park_box = new Pose2d(park_box_x * startSide, park_box_y * side_color, Math.toRadians(parkRed_box_heading));
            prePark_box = new Pose2d(prePark_box_x * startSide, prePark_box_y * side_color, Math.toRadians(parkRed_box_heading));
            yawOffset = 0;
        }else{
            boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(-135));
            park_box = new Pose2d(park_box_x * startSide, park_box_y * side_color, Math.toRadians(parkBlue_box_heading));
            prePark_box = new Pose2d(prePark_box_x * startSide, prePark_box_y * side_color, Math.toRadians(parkBlue_box_heading));
            yawOffset = Math.toRadians(-180);
        }
        park_chamber = new Pose2d(park_chamber_x * startSide,park_chamber_y * side_color,Math.toRadians(park_chamber_heading * side_color));

        if(side_color == RED){
            if(startSide == NEGATIVE){
                endPos = park_box;
            }else if(startSide == POSITIVE){
                endPos = park_chamber;
            }
        }else if(side_color == BLUE){
            if(startSide == POSITIVE){
                endPos = park_box;
            } else if (startSide == NEGATIVE) {
                endPos = park_chamber;
            }
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
        dropSamplePos = new Pose2d(dropSampleToHP_x * startSide, pre_intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        // TODO: change for blue
        intakeSpecimenPos_ground = new Pose2d(intakeSpecimen_ground_x,intakeSpecimen_ground_y * side_color, Math.toRadians(intakeSpecimenRed_ground_heading));

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

        upper.reset();
        upper.initialize();

        telemetry.addLine("init: trajectory");
        telemetry.update();
    }

    public void resetPoseforTest(){
        drive.setPoseEstimate(new Pose2d(48,-48,0));
        drive.update();
    }
    public void strafe(){
        drive.moveTo(new Pose2d(48,48,0),0);
        delay(5000);
        drive.moveTo(new Pose2d(48,-48,0),0);
    }

    protected void toOrigin(){
        upper.setWristIntake();
        upper.setArmPosition(0);
        delay(200);
        upper.setSlidePosition(0);
        delay(700);
    }

    protected void moveToDrop_sample(){
        drive.setSimpleMoveTolerance(0.6,0.6,Math.toRadians(3));
        drive.setSimpleMovePower(0.85);

        upper.setSlidePosition(0);
        upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
        upper.setSpinWristIntake_specimen();
        drive.moveTo(boxPos,correcting_time2);
        // Drop
        dropSample();
    }
    protected void moveToStartPos(){
        drive.moveTo(startPos, 500);
    }

    public void intakeSample(int count){
        drive.setSimpleMovePower(0.5);
        drive.setSimpleMoveTolerance(0.4,0.4,Math.toRadians(4));

        upper.setWristPreIntake();
        upper.setSpinWristIntake();
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        if(count == 1){
            drive.moveTo(intakeSamplePos_1, correcting_time2);
        } else if (count == 2) {
            drive.moveTo(intakeSamplePos_2, correcting_time2);
        }
        upper.setWristIntake();
        delay(200);
        upper.setClawGrab();
    }
    protected void intakeSample_3(){
        drive.setSimpleMovePower(0.6);
        drive.setSimpleMoveTolerance(0.4,0.4,Math.toRadians(4));
        upper.setWristPreIntake();
        upper.setArmPosition(SuperStructure.ARM_INTAKE);

        drive.moveTo(intakeSamplePos_3, correcting_time2);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
        delay(400);
        upper.setWristIntake();
        delay(200);
        upper.setClawGrab();
        delay(100);
    }

    protected void dropSample(){
        upper.setSlidePosition_verticle(SuperStructure.SLIDE_BOX_HIGH);
        delay(750);
        upper.setWristReleaseBox();
        delay(210);
        upper.setClawOpen();
    }

    protected void dropSampleToHP(){
        upper.setWristIntakeSpecimen();
        upper.setSpinWristIntake_specimen();
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        drive.moveTo(dropSamplePos,400);
        upper.setClawOpen();
    }

    //AutoRedChamber
    public void intake_release_RedSample(){
        upper.setWristPreIntake();
        drive.moveTo(intakeRedSample,0);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
        delay(200);
        upper.setWristIntake();
        delay(200);
        upper.setClawGrab();
        delay(100);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
        delay(50);
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        delay(50);
        upper.setClawOpen();
    }

    public void releaseRedSample(int count){
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
        if(count  == 1){
            drive.moveTo(releaseRedSample_1,0);
        } else if (count == 2) {
            drive.moveTo(releaseRedSample_2,0);
        } else if (count == 3) {
            drive.moveTo(releaseRedSample_3,0);
        }
        upper.setClawOpen();
        upper.setWristPreIntake();
    }

    protected void intakeSpecimen(){
        drive.setSimpleMoveTolerance(0.4,0.4,Math.toRadians(3));
        drive.setSimpleMovePower(0.3);
        drive.moveTo(intakeSpecimenPos,300);
        upper.setClawGrab();
    }
    protected void dropSpecimen_toIntakeSpecimen() {
        drive.setSimpleMovePower(0.95);
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(5));
        upper.setWristIntakeSpecimen();
        upper.setSpinWristIntake_specimen();
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        drive.moveTo(preIntakeSpecimenPos,0);

        drive.setSimpleMovePower(0.25);
        drive.setSimpleMoveTolerance(0.4,0.8,Math.toRadians(3));
        drive.moveTo(intakeSpecimenPos,300);
        upper.setClawGrab();
    }

    public void intakeSpecimen_ground(){
        drive.setSimpleMoveTolerance(0.5,0.5,Math.toRadians(2));
        drive.setSimpleMovePower(0.9);

        upper.setSlidePosition_horizontal(0);
        drive.moveTo(intakeSpecimenPos_ground,200);
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        delay(200);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
        delay(400);
        upper.setWristIntakeSpecimenGround();
        delay(300);
        upper.setClawGrab();
        delay(100);
        upper.setSlidePosition_horizontal(0);
    }

    public void moveToPreChamber(int count){
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(4));
        drive.setSimpleMovePower(1);
        upper.setWristPreIntake();
        upper.setArmPosition(SuperStructure.ARM_AUTO_CHAMBER);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_AUTO_CHAMBER);
        //upper.setSpinWristRelease_specimen();

        if(count == 1){
            drive.moveTo(preChamberPos,0);

        }else if (count == 2) {
            drive.moveTo(preChamberPos.plus(chamberPos_delta),0);

        } else if (count == 3) {
            drive.moveTo(preChamberPos.plus(chamberPos_delta).plus(chamberPos_delta),0);

        } else if (count == 4) {
            drive.moveTo(preChamberPos.plus(chamberPos_delta).plus(chamberPos_delta).plus(chamberPos_delta),0);

        } else if (count == 5) {
            drive.moveTo(preChamberPos.plus(chamberPos_delta).plus(chamberPos_delta).plus(chamberPos_delta).plus(chamberPos_delta),0);

        }
    }
    protected void releaseSpecimen(int count){
        upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);
        upper.setSlidePosition_verticle(SuperStructure.SLIDE_CHAMBER_HIGH);
        drive.setSimpleMovePower(0.3);
        drive.setSimpleMoveTolerance(0.3,0.3,Math.toRadians(2));

        if(count==1){
            drive.moveTo(chamberPos,200);
        } else if (count == 2) {
            drive.moveTo(chamberPos.plus(chamberPos_delta),200);
        } else if (count == 3) {
            drive.moveTo(chamberPos.plus(chamberPos_delta).plus(chamberPos_delta),200);
        } else if (count == 4) {
            drive.moveTo(chamberPos.plus(chamberPos_delta).plus(chamberPos_delta).plus(chamberPos_delta),200);
        } else if (count == 5) {
            drive.moveTo(chamberPos.plus(chamberPos_delta).plus(chamberPos_delta).plus(chamberPos_delta).plus(chamberPos_delta),335);
        }
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_CHAMBER_HIGH_DOWN);
        delay(300);
        upper.setClawOpen();
    }

    protected void dropSpecimen_toOrigin(){
        upper.setSlidePosition_verticle(SuperStructure.SLIDE_MIN);
        drive.moveTo(postChamberPos,0);
    }
    public void park_observation(){
        drive.setSimpleMovePower(1);
        upper.setArmPosition(0);
        upper.setSlidePosition_verticle(SuperStructure.SLIDE_MIN);
        drive.moveTo(park_chamber,500);
    }
    public void park(){
        upper.setArmPosition(0);
        upper.setSlidePosition_verticle(SuperStructure.SLIDE_MIN);
        Trajectory parkObservation = drive.trajectoryBuilder(chamberPos)
                .lineToLinearHeading(park_chamber)
                .build();
        drive.followTrajectory(parkObservation);
    }
    // TODO: only test for red now
    public void moveToPush(){
        TrajectorySequence moveToPush1 = drive.trajectorySequenceBuilder(new Pose2d(6.00, -35.60, Math.toRadians(90.00)))
                .splineTo(new Vector2d(16.06, -38.23), Math.toRadians(-9.20))
                .splineTo(new Vector2d(37.41, -20.86), Math.toRadians(90.0))
                .build();
        drive.followTrajectorySequence(moveToPush1);
    }

    public void pushSample(){
        upper.setClawOpen();
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
        delay(50);
        upper.setArmPosition(0);
        delay(50);
        drive.setSimpleMovePower(0.9);
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
        // Move to the first pre push pos
        drive.moveTo(pushSamplePos_midpoint,0);

        drive.moveTo(pushSamplePos_1,0);
        /*
        Trajectory moveToPushSamplePos_1 = drive.trajectoryBuilder(pushSamplePos_midpoint)
                        .lineToLinearHeading(pushSamplePos_1)
                        .build();
        */

        drive.moveTo(pushSamplePos_1.plus(pushSamplePos_delta),0);
        drive.moveTo(pushSamplePos_1,0);

        drive.moveTo(pushSamplePos_2,0);
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
        upper.setClawOpen();
        drive.moveTo(pushSamplePos_2.plus(pushSamplePos_delta),0);
    }


    public void park_box(){
        drive.moveTo(prePark_box,0);
        upper.setArmPosition(SuperStructure.ARM_HANG_AUTO);
        upper.setWristPreIntake();
        upper.setSpinWristIntake_specimen();
        upper.setSlidePosition_hang(400);
        drive.setSimpleMovePower(0.5);
        drive.moveTo(park_box,500);
    }
    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}