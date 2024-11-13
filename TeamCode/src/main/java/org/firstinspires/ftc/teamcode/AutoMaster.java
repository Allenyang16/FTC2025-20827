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
    Pose2d boxPos;
    Pose2d pushSamplePos_1;
    Pose2d pushSamplePos_2;


    protected void initHardware() throws InterruptedException{
        // TODO: must make sure that these poses are correct
        startPos = new Pose2d(12 * startSide ,52 * side_color,Math.toRadians(90 * side_color));
        boxPos = new Pose2d(-50 * startSide, 47 *side_color, Math.toRadians(55 * side_color));

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

        telemetry.addLine("init: trajectory");
        telemetry.update();
    }

    protected void toDrop_sample(){
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(boxPos,500);
    }

    protected void toPushSample(){
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(5));
        drive.setSimpleMovePower(0.9);
        drive.moveTo(boxPos,500);
    }


}
