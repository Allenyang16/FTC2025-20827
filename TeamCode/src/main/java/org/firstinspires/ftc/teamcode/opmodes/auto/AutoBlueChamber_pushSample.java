package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.AutoMaster;


@Autonomous

public class AutoBlueChamber_pushSample extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            moveToChamber(1);
            autoUpperToOrigin();

            pushSample(1);
            pushSample(2);
            pushSample(3);

            intakeSpecimen(1);
            moveToChamber(2);
            autoUpperToOrigin();
            dropSpecimen_toOrigin();

            intakeSpecimen(2);
            moveToChamber(3);
            autoUpperToOrigin();
            dropSpecimen_toOrigin();

            intakeSpecimen(3);
            moveToChamber(4);
            autoUpperToOrigin();
            dropSpecimen_toOrigin();

            intakeSpecimen(4);
            moveToChamber(5);
            autoUpperToOrigin();

            park_observation();
        }
    }
}
