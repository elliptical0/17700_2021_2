package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * @author TM
 */
@Autonomous(name="AutoOpMode_0", group="default")
public class AutoOpMode_0 extends BaseOpMode {
    public int state = 0;
    public int shotsFired = 0;

    public final double[] D = {0, 0, 0, 0};

    public final int STARTING_T_I = 0;

    private boolean b;

    public void changeState(int i) {
        state = i;
        clock.reset();
    }

    public final Transform[] transforms = {new Transform(36, 24, 0),
            new Transform(36, 24, Math.toRadians(-45)),
            new Transform(60, 24, 0),
            new Transform(72, 12, Math.toRadians(180)),
            new Transform(96, 36, Math.toRadians(180)),
            new Transform(120, 12, Math.toRadians(180)),
            SHOOTING_T
    };

    public void moveState(int i) {
        moveState(i, state + 1);
    }

    public void moveState(int i, int nextState) {
        changeState(nextState);
        return;
        /*
        wheelPowers = seekLocation(transform, transforms[i]);
        b = true;
        for(double p : wheelPowers) {
            if(Math.abs(p) > 0.05) {
                b = false;
            }
        }
        if(b) {
            changeState(nextState);
        }
         */
    }

    @Override
    public void initialize() {
        SharedVariables.startingTransform(STARTING_T_I);
        super.initialize();
    }

    @Override
    public void tick() {
        wheelPowers = D;
        switch(state) {
            case 0:
                startVision(true);
                moveState(0);
                break;
            case 1:
                moveState(1);
                break;
            case 2:
                if(cam.getFps() >= 0.0) {
                    changeState(3);
                }
                break;
            case 3:
                telemetry.addData("Stack Size:", filter.stackSize);
                telemetry.addData("Unfiltered Contour Count:", filter.findContoursOutput().size());
                telemetry.addData("Filtered Contour Count:", filter.filterContoursOutput().size());
                telemetry.addData("LastRatio:", filter.lastRatio);
                telemetry.addData("FPS:", cam.getFps());
                /*
                if(currentTime > 2) { //|| stackSize != 0) {
                    pauseVision();
                    changeState(4);
                }
                 */
                break;
            case 4:
                moveState(2);
                break;
            case 5:
                switch(filter.stackSize) {
                    case 1:
                        changeState(8);
                        break;
                    case 4:
                        changeState(9);
                        break;
                    default:
                        changeState(7);
                        break;
                }
            case 7:
                moveState(3, 10);
                break;
            case 8:
                moveState(4, 10);
            case 9:
                moveState(5, 10);
            case 10:
                if(!SERVOS_ACTIVE) {
                    changeState(2);
                    break;
                }
                wobbleAimIndex = 2;
                if(servoAtPos(wobbleAim, WOBBLE_AIM_POSITIONS[2])) {
                    changeState(11);
                }
                break;
            case 11:
                wobbleHandIndex = 0;
                if(servoAtPos(wobbleHand, WOBBLE_HAND_POSITIONS[0])) {
                    wobbleAimIndex = 0;
                }
                if(servoAtPos(wobbleAim, WOBBLE_AIM_POSITIONS[0])) {
                    changeState(12);
                }
                break;
            case 12:
                moveState(6);
                break;
            case 13:
                if(shotsFired < 3) {
                    powerIntake(false, true, false);
                    if(currentTime > 1) {
                        changeState(14);
                    }
                } else {
                    changeState(99);
                }
                break;
            case 14:
                powerIntake(false, true, true);
                if(currentTime > 0.5) {
                    shotsFired++;
                    changeState(13);
                }
                break;
            case 99:
                SharedVariables.transform = transform;
                stop();
                break;
        }
        updateMotors();
        telemetry.addData("State:", state);
        updateTelemetry();
    }
}
