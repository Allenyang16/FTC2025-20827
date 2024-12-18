package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoBlueBox extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = POSITIVE;

        // TODO: Check the init of positions
        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            moveToDropFirst_sample();
            toOrigin();

            intakeSample_1();
            moveToDrop_sample();
            toOrigin();

            intakeSample_2();
            moveToDrop_sample();
            toOrigin();

            intakeSample_3();
            moveToDrop_sample();
            toOrigin();
            park_box();
        }
    }
}
