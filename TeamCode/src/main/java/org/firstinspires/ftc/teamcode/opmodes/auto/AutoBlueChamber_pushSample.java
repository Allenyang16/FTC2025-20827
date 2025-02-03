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
            //releaseSpecimen(1);

            grabSample(1);
            grabSample(2);
            //grabSample(3);

            intakeSpecimen();
            moveToChamber(2);
            releaseSpecimen(2);

            dropSpecimen_toIntakeSpecimen(3);
            moveToChamber(3);
            releaseSpecimen(3);

            dropSpecimen_toIntakeSpecimen(4);
            moveToChamber(4);
            releaseSpecimen(4);

            //dropSpecimen_toIntakeSpecimen(5);
            //moveToPreChamber(5);
            //releaseSpecimen(5);

            park_observation();
        }
    }
}
