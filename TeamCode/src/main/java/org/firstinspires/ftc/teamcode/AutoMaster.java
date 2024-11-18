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

    Pose2d startPos;
    public static double startPos_x = 12.125, startPos_y = 59, startPos_heading = -90;

    Pose2d boxPos;
    public static double box_x = 56.5, box_y = 53, box_heading = -45;

    Pose2d chamberPos;

    // TODO: TEST VALUE of all these poses
    Pose2d intakeSamplePos_1;
    public static double intake_samplePos1_x = 55, intake_samplePos1_y = 50, intake_samplePos1_heading = -105; // Degree
    Pose2d intakeSamplePos_2;
    public static double intake_samplePos2_x = 57, intake_samplePos2_y = 48, intake_samplePos2_heading = -90;
    Pose2d intakeSamplePos_3;
    public static double intake_samplePos3_x = 55, intake_samplePos3_y = 50, intake_samplePos3_heading = -105;

    Pose2d pushSamplePos_1;
    Pose2d pushSamplePos_2;
    Pose2d pushSamplePos_3;

    Pose2d intakeSpecimenPos;


    protected void initHardware() throws InterruptedException{
        // TODO: must make sure that these poses are correct
        startPos = new Pose2d(startPos_x * startSide ,startPos_y * side_color,Math.toRadians(startPos_heading * side_color));
        boxPos = new Pose2d(box_x * startSide, box_y * side_color, Math.toRadians(box_heading * side_color));

        intakeSamplePos_1 = new Pose2d(intake_samplePos1_x * startSide, intake_samplePos1_y * side_color, Math.toRadians(intake_samplePos1_heading * side_color));
        intakeSamplePos_2 = new Pose2d(intake_samplePos2_x * startSide, intake_samplePos2_y * side_color, Math.toRadians(intake_samplePos2_heading * side_color));
        intakeSamplePos_3 = new Pose2d(intake_samplePos3_x * startSide, intake_samplePos3_y * side_color, Math.toRadians(intake_samplePos3_heading * side_color));

        pushSamplePos_1 = new Pose2d(-40 * startSide, 40 * side_color, Math.toRadians(-90 * side_color));

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
        upper.setSlidePosition(0);
        delay(1000);
        upper.setArmPosition(0);
    }

    protected void moveToDrop_sample(){
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(boxPos,1500);

        // Drop
        dropSample();
    }

    protected void intakeSample_1(){
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSamplePos_1,500);
        delay(300);
        upper.setClawGrab();
        delay(200);
    }
    protected void intakeSample_2(){
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSamplePos_1,500);
        delay(300);
        upper.setClawGrab();
        delay(200);
    }
    protected void intakeSample_3(){
        upper.setArmPosition(SuperStructure.ARM_INTAKE);
        drive.moveTo(intakeSamplePos_1,500);
        delay(300);
        upper.setClawGrab();
        delay(200);
    }

    protected void dropSample(){
        upper.setArmPosition(SuperStructure.ARM_RELEASE);
        upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
        upper.setWristReleaseBox();
        delay(1500);
        upper.setClawOpen();
        delay(200);
    }

    protected void toPushSample(){
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(boxPos,500);
    }

    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}
