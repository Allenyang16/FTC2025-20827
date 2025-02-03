package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous @Disabled
public class AutoRedChamber extends AutoMaster {
    @Override

    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = POSITIVE;

        initHardware();
        while (opModeInInit()){
//            resetPosAndIMU();
        }


        if(opModeIsActive()){
            releaseSpecimen(1);
            dropSpecimen_toOrigin();

            intake_release_RedSample();
            releaseRedSample(1);

            intakeSpecimen_ground();
            releaseSpecimen(2);
        }
    }
}
