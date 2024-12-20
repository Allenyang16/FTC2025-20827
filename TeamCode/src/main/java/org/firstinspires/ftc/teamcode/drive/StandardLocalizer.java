package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.GeomUtil;

/*
 * Tracking wheel localizer implementation assuming the configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    |           || |
 *    |           || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardLocalizer implements Localizer {
    public double xOffset = -132, yOffset = 133;
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    private Pose2d poseVelocity = new Pose2d(0, 0, 0);

    private Pose2D currentPos = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0);
    private Pose2D currentVelocity = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0);

    private final NanoClock time;
    GoBildaPinpointDriver odometry;

    public StandardLocalizer(HardwareMap hardwareMap) {
        odometry = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odometry.setOffsets(xOffset,yOffset);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        time = NanoClock.system();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @NonNull
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
        currentPos = odometry.getPosition();
        double current_x = currentPos.getX(DistanceUnit.INCH);
        double current_y = currentPos.getY(DistanceUnit.INCH);
        double rotation = currentPos.getHeading(AngleUnit.RADIANS);

        currentVelocity =  odometry.getVelocity();
        double velocity_x = currentVelocity.getX(DistanceUnit.INCH);
        double velocity_y = currentVelocity.getY(DistanceUnit.INCH);
        double velocity_heading = odometry.getHeadingVelocity();

        poseEstimate = new Pose2d(current_x,current_y,rotation);
        poseVelocity = new Pose2d(velocity_x,velocity_y,velocity_heading);
        odometry.update();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d poseEstimate) {
        this.poseEstimate = poseEstimate;
        odometry.setPosition(new Pose2D(DistanceUnit.INCH,poseEstimate.getX(),poseEstimate.getY(),AngleUnit.RADIANS,poseEstimate.getHeading()));
        odometry.update();
    }
}