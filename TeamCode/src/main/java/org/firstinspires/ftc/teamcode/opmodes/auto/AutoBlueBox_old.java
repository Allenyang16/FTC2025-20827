package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoBlueBox_old extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = POSITIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){

            moveToDrop_sample1(1);
            toOrigin();

            intakeSample(1);
            moveToDrop_sample1(2);
            toOrigin();

            intakeSample(2);
            moveToDrop_sample1(3);
            toOrigin();

            intakeSample_3();
            moveToDrop_sample1(4);
            toOrigin();


            park_box();
        }
    }
}

