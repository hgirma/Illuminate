package org.firstinspires.ftc.teamcode.octobotOld;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class OctobotAuto1_3 extends OctobotAutoEssentials {
    public OctobotAuto1_3() {
        super(560, 10);
    }

    @Override
    public void runOpMode() {
        initialize();
        initIMU();
        waitForStart();
        goRotations(4, 0.5);
        strafeLeft(2, 0.5);

    }
}