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

            dropSampletoIntakeSample(1);
            moveToDrop_sample(1);

            dropSampletoIntakeSample(2);
            moveToDrop_sample(2);

            dropSampleToIntakeSample_3();
            moveToDrop_sample(3);

/*
            runToFieldToGrabSample();
            recognizeSample();
            moveToDrop_sample(4);
 */

            park_box();
        }
    }
}
