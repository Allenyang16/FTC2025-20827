package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.AutoMaster;


@Autonomous

public class AutoRedChamber_pushSample extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = POSITIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            moveToChamber(1);
            releaseSpecimen(1);

            pushSample(1);
            pushSample(2);
            pushSample(3);

            intakeSpecimen();
            moveToChamber(2);
            releaseSpecimen(2);
            dropSpecimen_toOrigin();

            intakeSpecimen();
            moveToChamber(3);
            releaseSpecimen(3);
            dropSpecimen_toOrigin();

            intakeSpecimen();
            moveToChamber(4);
            releaseSpecimen(4);
            dropSpecimen_toOrigin();

            intakeSpecimen();
            moveToChamber(5);
            releaseSpecimen(5);

            park_observation();
        }
    }
}
