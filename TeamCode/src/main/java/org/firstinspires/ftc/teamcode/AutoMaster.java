package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
//import org.firstinspires.ftc.teamcode.testings.DetectPipeline;
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

    public static int correcting_time = 100, correcting_time2 = 200;
    public static double yawOffset = 0;

    Pose2d startPos;
    public static double startPos_x = 39, startPos_y = 62, startPos_heading = -90;
    public static double startPos_chamber_x = 9, startPos_box_x = 39;

    Pose2d boxPos;
    public static double box_x = 56, box_y = 58, boxRed_heading = 45, boxBlue_heading = -135; // or 135 in blue

    Pose2d chamberPos;
    public static double chamber_x = 9, chamber_y = 32.5, chamber_heading = -90;
    Pose2d chamberPos2;
    public static double chamber2_x = 8;
    Pose2d chamberPos_delta;
    Pose2d chamberPos_delta2;
    public static double chamber_delta_x = 1;
    public static double chamber_delta_x2 = 2.5;

    Pose2d preChamberPos;
    public static double preChamber_x = 9, preChamber_y = 50;
    Pose2d preChamberPos1;
    public static double preChamber1_x = 6, preChamber1_y = 37;

    Pose2d postChamberPos;
    public static double postChamber_x = 35, postChamber_y = 40;

    Pose2d intakeSamplePos_1;
    public static double intake_samplePos1_x = 47, intake_samplePos1_y = 42, intake_samplePos1_heading = -90; // Degree
    Pose2d intakeSamplePos_2;
    public static double intake_samplePos2_x = 57, intake_samplePos2_y = 42, intake_samplePos2_heading = -90;
    Pose2d intakeSamplePos_3;
    public static double intake_samplePos3_x = 48, intake_samplePos3_y = 44.5, intake_samplePos3_heading = -130;
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


    Pose2d grabSamplePos_1;
    Pose2d grabSamplePos_2;
    Pose2d grabSamplePos_3;
    Pose2d pushSamplePos_1;
    Pose2d pushSamplePos_2;
    Pose2d pushSamplePos_3;
    Pose2d pushSamplePos_midpoint1;
    Pose2d pushSamplePos_midpoint2;
    Pose2d throwSamplePos_1;
    Pose2d throwSamplePos_2;
    Pose2d throwSamplePos_3;
    Pose2d pushSamplePos_delta;

    public static double grabSample1_x = 25.5, grabSample2_x = 34.7, grabSample3_x = 46.9;
    public static double grabSample_y = 29;
    public static double pushSample1_x = 49, pushSample2_x = 58, pushSample3_x = 64, midpoint_x = 38;
    public static double pushSample_y = 16, midpoint1_y = 36, pushSample_heading = -90, pushSample_delta_y = 32;
    public static double throwSample_y = 55;
    public static double grabSample_heaidng = -10.94, throwSample_heading = 30;

    Pose2d intakeSpecimenPos;
    public static double intakeSpecimen_x = 40, intakeSpecimen_y = 57.2, intakeSpecimen_heading = -90;
    Pose2d intakeSpecimenPrePos_ground;
    public static double pre_intakeSpecimen_x_ground = 22, pre_intakeSpecimen_y_ground = 43;
    Pose2d intakeSpecimenPos_ground;
    public static double intakeSpecimen_ground_x = 24, intakeSpecimen_ground_y = 45, intakeSpecimen_ground_heading = 50;

    Pose2d preIntakeSpecimenPos_wall;
    public static double pre_intakeSpecimen_y_wall = 52;
    Pose2d dropSamplePos;
    public static double dropSampleToHP_x = 50;

    //Getting Sample From the Field
    public static Pose2d runToField = new Pose2d(-24,-10,Math.toRadians(90));
    public static double runToField_x = 24, runToField_y = 10, runToFieldRed_heading = 0, runToFieldBlue_heading = 180;

    public static Pose2d park_box = new Pose2d(-24,-10,Math.toRadians(90));
    public static double park_box_x = 24, park_box_y = 10, parkRed_box_heading = 0, parkBlue_box_heading = 180;
    public static Pose2d prePark_box;
    public static double prePark_box_x = 45, prePark_box_y = 10;

    public static double park_chamber_x = 53, park_chamber_y = 57.2, park_chamber_heading = -90;
    public static Pose2d park_chamber = new Pose2d(park_chamber_x, park_chamber_y,Math.toRadians(park_chamber_heading));


    public static Pose2d endPos = park_box;


    protected void initHardware() throws InterruptedException{
//        DetectPipeline pipeline = new DetectPipeline();
//        drive.camera.setPipeline(pipeline);

        if(side_color == RED && startSide == POSITIVE){
            startPos_x = startPos_chamber_x;
        } else if (side_color == BLUE && startSide == NEGATIVE) {
            startPos_x = startPos_chamber_x;
        } else{
            startPos_x = startPos_box_x;
        }

        startPos = new Pose2d(startPos_x * startSide ,startPos_y * side_color,Math.toRadians(startPos_heading * side_color));
        if(side_color == RED){
            boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(boxRed_heading));
            park_box = new Pose2d(park_box_x * startSide, park_box_y * side_color, Math.toRadians(parkRed_box_heading));
            prePark_box = new Pose2d(prePark_box_x * startSide, prePark_box_y * side_color, Math.toRadians(parkRed_box_heading));
            runToField = new Pose2d(runToField_x * startSide, runToField_y * side_color, Math.toRadians(runToFieldRed_heading));
            yawOffset = 0;
        }else{
            boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(boxBlue_heading));
            park_box = new Pose2d(park_box_x * startSide, park_box_y * side_color, Math.toRadians(parkBlue_box_heading));
            prePark_box = new Pose2d(prePark_box_x * startSide, prePark_box_y * side_color, Math.toRadians(parkBlue_box_heading));
            runToField = new Pose2d(runToField_x * startSide, runToField_y * side_color, Math.toRadians(runToFieldBlue_heading));
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
        chamberPos_delta2 = new Pose2d(chamber_delta_x2 * startSide, 0, 0);

        preChamberPos = new Pose2d(preChamber_x * startSide, preChamber_y * side_color, Math.toRadians(chamber_heading * side_color));
        preChamberPos1 = new Pose2d(preChamber1_x * startSide, preChamber1_y * side_color, Math.toRadians(chamber_heading * side_color));
        postChamberPos = new Pose2d(postChamber_x * startSide, postChamber_y * side_color, Math.toRadians(chamber_heading * side_color));

        intakeSamplePos_1 = new Pose2d(intake_samplePos1_x * startSide, intake_samplePos1_y * side_color, Math.toRadians(intake_samplePos1_heading * side_color));
        intakeSamplePos_2 = new Pose2d(intake_samplePos2_x * startSide, intake_samplePos2_y * side_color, Math.toRadians(intake_samplePos2_heading * side_color));
        if(startSide == POSITIVE){
            intakeSamplePos_3 = new Pose2d(intake_samplePos3_x * startSide, intake_samplePos3_y * side_color, Math.toRadians(sample3_positive_heading * side_color));
        }else{
            intakeSamplePos_3 = new Pose2d(intake_samplePos3_x * startSide, intake_samplePos3_y * side_color, Math.toRadians(intake_samplePos3_heading * side_color));
        }

        intakeSpecimenPos = new Pose2d(intakeSpecimen_x * startSide, intakeSpecimen_y * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        preIntakeSpecimenPos_wall = new Pose2d(intakeSpecimen_x * startSide, pre_intakeSpecimen_y_wall * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        dropSamplePos = new Pose2d(dropSampleToHP_x * startSide, pre_intakeSpecimen_y_wall * side_color, Math.toRadians(intakeSpecimen_heading * side_color));
        // TODO: change for blue
        intakeSpecimenPos_ground = new Pose2d(intakeSpecimen_ground_x * startSide,intakeSpecimen_ground_y * side_color, Math.toRadians(intakeSpecimen_ground_heading * side_color));
        intakeSpecimenPrePos_ground = new Pose2d(pre_intakeSpecimen_x_ground * startSide, pre_intakeSpecimen_y_ground * side_color, Math.toRadians(intakeSpecimen_ground_heading * side_color));
        grabSamplePos_1 = new Pose2d(grabSample1_x * startSide, grabSample_y * side_color, Math.toRadians(grabSample_heaidng * side_color));
        grabSamplePos_2 = new Pose2d(grabSample2_x * startSide, grabSample_y * side_color, Math.toRadians(grabSample_heaidng * side_color));
        grabSamplePos_3 = new Pose2d(grabSample3_x * startSide, grabSample_y * side_color, Math.toRadians(grabSample_heaidng * side_color));
        pushSamplePos_1 = new Pose2d(pushSample1_x * startSide, pushSample_y * side_color, Math.toRadians(pushSample_heading  * side_color) );
        pushSamplePos_2 = new Pose2d(pushSample2_x * startSide, pushSample_y * side_color, Math.toRadians(pushSample_heading  * side_color) );
        pushSamplePos_3 = new Pose2d(pushSample3_x * startSide, pushSample_y * side_color, Math.toRadians(pushSample_heading * side_color) );
        pushSamplePos_midpoint1 = new Pose2d(midpoint_x * startSide, midpoint1_y * side_color, Math.toRadians(pushSample_heading  * side_color) );
        pushSamplePos_midpoint2 = new Pose2d(midpoint_x * startSide, pushSample_y * side_color, Math.toRadians(pushSample_heading  * side_color) );
        throwSamplePos_1 = new Pose2d(grabSample1_x * startSide, throwSample_y * side_color, Math.toRadians(throwSample_heading * side_color));
        throwSamplePos_2 = new Pose2d(grabSample2_x * startSide, throwSample_y * side_color, Math.toRadians(throwSample_heading * side_color));
        throwSamplePos_3 = new Pose2d(grabSample3_x * startSide, throwSample_y * side_color, Math.toRadians( throwSample_heading * side_color));
        pushSamplePos_delta = new Pose2d(0, pushSample_delta_y * side_color, 0);

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

    public void resetPosAndIMU(){
        drive.resetPosAndIMU();
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
        delay(100);
        upper.setSlidePosition(0);
        delay(700);
    }

    protected void moveToDrop_sample(){
        upper.setArmPosition(SuperStructure.ARM_RELEASE_BOX);
        upper.setSpinWristIntake_specimen();

        upper.setSlidePosition_verticle(SuperStructure.SLIDE_BOX_HIGH);
        upper.setWristReleaseBox();
        delay(200);
        drive.setSimpleMoveTolerance(0.8,0.8,Math.toRadians(3));
        drive.setSimpleMovePower(0.5);
        drive.moveTo(boxPos, correcting_time2);

        upper.setClawOpen();
        delay(100);
    }
    protected void moveToStartPos(){
        drive.moveTo(startPos, 500);
    }

    public void intakeSample(int count){
        drive.setSimpleMovePower(0.5);
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));

        upper.setWristPreIntake();
        upper.setSpinWristIntake();
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        if(count == 1){
            drive.moveTo(intakeSamplePos_1, correcting_time2);
        } else if (count == 2) {
            drive.moveTo(intakeSamplePos_2, correcting_time2);
        }
        upper.setWristIntake();
        delay(150);
        upper.switchClawState();


    }
    protected void intakeSample_3(){
        drive.setSimpleMovePower(0.5);
        drive.setSimpleMoveTolerance(0.8,0.8,Math.toRadians(5));
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSamplePos_3, correcting_time2);

        upper.setSpinWristAuto();
        upper.setWristIntake();
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
        delay(400);
        upper.setClawGrab();
        delay(200);
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

    protected void intakeSpecimen(int count){
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        upper.setSlidePosition(SuperStructure.SLIDE_AUTO);
        upper.setSpinWristIntake_specimen();
        upper.setWristIntakeSpecimen();
        if(count == 1){
            drive.setSimpleMoveTolerance(1,1,Math.toRadians(3));
            drive.setSimpleMovePower(0.6);
            drive.moveTo(preIntakeSpecimenPos_wall,0);
            drive.setSimpleMovePower(0.45);
            drive.setSimpleMoveTolerance(0.8,0.8,Math.toRadians(1));
            drive.moveTo(intakeSpecimenPos,50);
        } else {
            drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
            drive.setSimpleMovePower(1);
            drive.moveTo(preIntakeSpecimenPos_wall,0);
            drive.setSimpleMovePower(0.45);
            drive.setSimpleMoveTolerance(0.8,0.8,Math.toRadians(1));
            drive.moveTo(intakeSpecimenPos,50);
        }
        //delay(200);
        upper.setClawGrab();
        delay(50);
    }

    protected void dropSpecimen_toIntakeSpecimen(int count) {

        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        upper.setWristIntakeSpecimen();
        upper.setSpinWristIntake_specimen();
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
        delay(100);
        upper.setArmPosition(SuperStructure.ARM_INTAKE_SPECIMEN);
        drive.moveTo(preIntakeSpecimenPos_wall,0);

        /*
        if(count == 3){
            TrajectorySequence dropSpecimen_toIntakeSpecimen1 = drive.trajectorySequenceBuilder(new Pose2d(9 * startSide, 37 * side_color, Math.toRadians(-90 * side_color)))
                    .lineToConstantHeading(new Vector2d(intakeSpecimen_x * startSide, pre_intakeSpecimen_y * side_color))
                    .build();
            drive.followTrajectorySequence(dropSpecimen_toIntakeSpecimen1);

        } else if (count == 4) {
            TrajectorySequence dropSpecimen_toIntakeSpecimen2 = drive.trajectorySequenceBuilder(new Pose2d(10 * startSide, 37 * side_color, Math.toRadians(-90 * side_color)))
                    .lineToConstantHeading(new Vector2d(intakeSpecimen_x * startSide, pre_intakeSpecimen_y * side_color))
                    .build();
            drive.followTrajectorySequence(dropSpecimen_toIntakeSpecimen2);

        }else {
            TrajectorySequence dropSpecimen_toIntakeSpecimen3 = drive.trajectorySequenceBuilder(new Pose2d(11 * startSide, 37 * side_color, Math.toRadians(-90 * side_color)))
                .lineToConstantHeading(new Vector2d(intakeSpecimen_x * startSide, pre_intakeSpecimen_y * side_color))
                .build();
            drive.followTrajectorySequence(dropSpecimen_toIntakeSpecimen3);

        }
        */

        drive.setSimpleMovePower(0.3);
        drive.setSimpleMoveTolerance(0.8,0.8,Math.toRadians(1));
        drive.moveTo(intakeSpecimenPos,50);
        upper.setClawGrab();

        /*
        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(0.6,0.6,Math.toRadians(1));
        upper.setWristIntakeSpecimenGround();
        delay(100);
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSpecimenPos_ground,100);
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
        delay(200);
        upper.setClawGrab();
        delay(50);
        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
        delay(100);
        */
    }

    public void intakeSpecimen_ground(){
        upper.setSlidePosition(SuperStructure.SLIDE_MIN);
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
        drive.setSimpleMovePower(1);
        upper.setSpinWristIntake_specimen();
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSpecimenPrePos_ground,50);
        drive.setSimpleMovePower(0.3);
        drive.setSimpleMoveTolerance(0.8,0.8,Math.toRadians(1));
        upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
        upper.setWristIntakeSpecimenGround();
        delay(100);
        drive.moveTo(intakeSpecimenPos_ground,100);
        delay(100);
        upper.setClawGrab();
        delay(100);
        upper.setSlidePosition_horizontal(0);
    }

    public void moveToChamber(int count){
        upper.setWristReleaseChamber();
        upper.setArmPosition(SuperStructure.ARM_RELEASE_CHAMBER);

        if(count == 1) {
            upper.setSpinWristRelease_specimen();
            drive.setSimpleMovePower(0.9);
            drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
            upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_AUTO);
            drive.moveTo(chamberPos,0);
            delay(25);

        }else if (count == 2) {
            drive.setSimpleMovePower(1);
            drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
            upper.setSpinWristRelease_specimen();
            drive.moveTo(preChamberPos.plus(chamberPos_delta),50);
            delay(25);
            upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
            drive.moveTo(chamberPos.plus(chamberPos_delta),0);
            delay(25);
           /* TrajectorySequence releaseSpecimen2 = drive.trajectorySequenceBuilder(new Pose2d(40 * startSide, 58 * side_color, Math.toRadians(-90 * side_color)))
                    .lineToConstantHeading(new Vector2d(9 * startSide, 39.5 * side_color))
                    .build();
            drive.followTrajectorySequence(releaseSpecimen2);*/


        } else if (count == 3) {
            drive.setSimpleMovePower(1);
            drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
            upper.setSpinWristRelease_specimen();
            drive.moveTo(preChamberPos.plus(chamberPos_delta).plus(chamberPos_delta),50);
            delay(25);
            upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
            drive.moveTo(chamberPos.plus(chamberPos_delta).plus(chamberPos_delta),0);
            delay(25);
            /*TrajectorySequence releaseSpecimen3 = drive.trajectorySequenceBuilder(new Pose2d(40 * startSide, 58 * side_color, Math.toRadians(-90 * side_color)))
                    .lineToConstantHeading(new Vector2d(10 * startSide, 37 * side_color))
                    .build();
            drive.followTrajectorySequence(releaseSpecimen3);*/

        } else if (count == 4) {
            drive.setSimpleMovePower(1);
            drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
            upper.setSpinWristRelease_specimen();
            drive.moveTo(preChamberPos.minus(chamberPos_delta2),50);
            delay(25);
            upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
            drive.moveTo(chamberPos.minus(chamberPos_delta2),0);
            delay(25);
            /*
            TrajectorySequence releaseSpecimen4 = drive.trajectorySequenceBuilder(new Pose2d(40 * startSide, 58 * side_color, Math.toRadians(-90 * side_color)))
                    .lineToConstantHeading(new Vector2d(11 * startSide, 37 * side_color))
                    .build();
            drive.followTrajectorySequence(releaseSpecimen4);
            */

        } else if (count == 5) {
            drive.setSimpleMovePower(1);
            drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
            upper.setSpinWristRelease_specimen();
            drive.moveTo(preChamberPos.minus(chamberPos_delta2).minus(chamberPos_delta2),50);
            delay(25);
            upper.setSlidePosition(SuperStructure.SLIDE_CHAMBER_HIGH_TELEOP);
            drive.moveTo(chamberPos.minus(chamberPos_delta2).minus(chamberPos_delta2),0);
            delay(25);

            /*TrajectorySequence releaseSpecimen5 = drive.trajectorySequenceBuilder(new Pose2d(40 * startSide, 58 * side_color, Math.toRadians(-90 * side_color)))
                    .lineToConstantHeading(new Vector2d(12 * startSide, 37 * side_color))
                    .build();
            drive.followTrajectorySequence(releaseSpecimen5);*/

        }
    }
    protected void autoUpperToOrigin(){
        //delay(200);
        upper.setClawOpen();
        delay(25);
        upper.setSlidePosition_verticle(SuperStructure.SLIDE_MIN);
        delay(40);
        upper.setArmPosition(0);
        upper.setWristPreIntake();
        upper.setSpinWristIntake();
    }

    protected void dropSpecimen_toOrigin(){
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

    public void grabSample(int count){
        drive.setSimpleMoveTolerance(0.5,0.5,Math.toRadians(3));
        drive.setSimpleMovePower(1);

        if (count == 1){

            drive.moveTo(grabSamplePos_1,200);
            upper.setArmPosition(SuperStructure.ARM_INTAKE);
            delay(175);
            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
//            upper.setSpinWristIntake_spinClockwise();
//            upper.setWristIntake();
            upper.setWristPreIntake();
            upper.setSpinWristIntake();
            delay(250);
//            upper.setClawGrab();
//            delay(50);
//            upper.setSlidePosition(SuperStructure.SLIDE_MIN);
//            delay(100);
//            upper.setWristPreIntake();
            drive.moveTo(throwSamplePos_1,0);
            upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
//            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
//            upper.setWristPreIntake();
//            upper.setSpinWristIntake();
//            delay(200);
//            upper.setClawOpen();
            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
        } else if (count == 2){
            drive.moveTo(grabSamplePos_2,200);
            upper.setArmPosition(SuperStructure.ARM_INTAKE);
            delay(175);
            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
//            upper.setSpinWristIntake_spinClockwise();
//            upper.setWristIntake();
            upper.setWristPreIntake();
            upper.setSpinWristIntake();
            delay(250);
//            upper.setClawGrab();
//            delay(50);
//            upper.setSlidePosition(SuperStructure.SLIDE_MIN);
//            delay(100);
//            upper.setWristPreIntake();
            drive.moveTo(throwSamplePos_2,0);
            upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
//            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
//            upper.setWristPreIntake();
//            upper.setSpinWristIntake();
//            delay(200);
//            upper.setClawOpen();
            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);

        } else{
            drive.moveTo(grabSamplePos_3,150);
            upper.setArmPosition(SuperStructure.ARM_INTAKE);
            delay(175);
            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
//            upper.setSpinWristIntake_spinClockwise();
//            upper.setWristIntake();
            upper.setWristPreIntake();
            upper.setSpinWristIntake();
            delay(250);
//            upper.setClawGrab();
//            delay(50);
//            upper.setSlidePosition(SuperStructure.SLIDE_MIN);
//            delay(100);
//            upper.setWristPreIntake();
            drive.moveTo(throwSamplePos_3,0);
            upper.setArmPosition(SuperStructure.ARM_PRE_INTAKE);
//            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_INTAKE_MAX);
//            upper.setWristPreIntake();
//            upper.setSpinWristIntake();
//            delay(200);
//            upper.setClawOpen();
            upper.setSlidePosition_horizontal(SuperStructure.SLIDE_MIN);
        }
    }

    public void pushSample(int count){
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        if (count == 1){
            drive.moveTo(pushSamplePos_midpoint1,0);
            drive.moveTo(pushSamplePos_midpoint2,0);
            drive.moveTo(pushSamplePos_1,0);
            drive.moveTo(pushSamplePos_1.plus(pushSamplePos_delta),0);
        }
        else if (count == 2){
            drive.moveTo(pushSamplePos_1,0);
            drive.moveTo(pushSamplePos_2,0);
            drive.moveTo(pushSamplePos_2.plus(pushSamplePos_delta),0);
        }
        else{
            drive.moveTo(pushSamplePos_2,0);
            drive.moveTo(pushSamplePos_3,0);
            drive.moveTo(pushSamplePos_3.plus(pushSamplePos_delta),0);
        }
    }

    public void runToFieldToGrabSample(){
        drive.moveTo(runToField,0);
        upper.setArmPosition(0);
        upper.setSlidePosition(0);
        upper.setWristPreIntake();
        upper.setSpinWristIntake();
        upper.setClawOpen();
    }

    public void recognizeSample(){
//        double cameraAngle = drive.getCameraAngle();
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