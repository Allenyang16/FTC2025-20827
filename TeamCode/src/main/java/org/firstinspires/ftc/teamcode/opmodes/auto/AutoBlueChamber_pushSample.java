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
            releaseSpecimen(1);

            grabSample(1);
            grabSample(2);
            grabSample(3);

            intakeSpecimen();
            moveToPreChamber(2);
            releaseSpecimen(2);

            dropSpecimen_toIntakeSpecimen();
            moveToPreChamber(3);
            releaseSpecimen(3);

            dropSpecimen_toIntakeSpecimen();
            moveToPreChamber(4);
            releaseSpecimen(4);

            dropSpecimen_toIntakeSpecimen();
            moveToPreChamber(5);
            releaseSpecimen(5);

            park_observation();
        }
    }
}
