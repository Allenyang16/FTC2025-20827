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
            moveToPreChamber(1);
            delay(100);

            grabSample(1);
            delay(100);
            grabSample(2);
            delay(100);
            grabSample(3);

            intakeSpecimen();
            moveToPreChamber(2);
            delay(50);
            //releaseSpecimen(1);

            dropSpecimen_toIntakeSpecimen(3);
            moveToPreChamber(3);
            delay(50);
            //releaseSpecimen(2);

            dropSpecimen_toIntakeSpecimen(4);
            moveToPreChamber(4);
            delay(50);
            //releaseSpecimen(3);

            dropSpecimen_toIntakeSpecimen(5);
            moveToPreChamber(5);
            delay(50);
            //releaseSpecimen(4);

            park_observation();
        }
    }
}
