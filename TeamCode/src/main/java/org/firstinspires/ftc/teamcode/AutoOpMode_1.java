package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoOpMode_1", group="default")
public class AutoOpMode_1 extends AutoOpMode_0 {

    @Override
    public void initialize() {
        VISION_ENABLED = false;
        super.initialize();
        resetPosition(0);
    }
}