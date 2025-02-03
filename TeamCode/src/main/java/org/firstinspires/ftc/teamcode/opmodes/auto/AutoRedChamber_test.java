package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedChamber_test extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = POSITIVE ;

        initHardware();
        while (opModeInInit()){


        }

        if(opModeIsActive()){
            moveToChamber(1);
            releaseSpecimen(1);

            grabSample(1);
            grabSample(2);
            grabSample(3);

            intakeSpecimen_ground();
            moveToChamber(2);
            releaseSpecimen(2);

            intakeSpecimen_ground();
            moveToChamber(3);
            releaseSpecimen(3);

            intakeSpecimen_ground();
            moveToChamber(4);
            releaseSpecimen(4);

            intakeSpecimen_ground();
            moveToChamber(5);
            releaseSpecimen(5);

            //dropSpecimen_toIntakeSpecimen(5);
            //moveToPreChamber(5);
            //releaseSpecimen(5);

            park_observation();
        }
    }
}
