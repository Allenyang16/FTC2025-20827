package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedBox extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = NEGATIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            moveToDrop_sample(0);
            toOrigin();

            intakeSample(1);
            moveToDrop_sample(1);
            toOrigin();

            intakeSample(2);
            moveToDrop_sample(2);
            toOrigin();

            intakeSample_3();
            moveToDrop_sample(3);
            toOrigin();

/*
            runToFieldToGrabSample();
            recognizeSample();
            moveToDrop_sample(4);
            toOrigin();
 */

            park_box();
        }
    }
}
