package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.MyMathLib.*;

/**
 * @author TM
 */
@Autonomous(name="AutoOpMode_0", group="default")
public class AutoOpMode_0 extends BaseOpMode {
    public int state = 0;
    public int shotsFired = 0;

    public final double[] D = {0, 0, 0, 0};

    public final int STARTING_T_I = 0;

    public double stateStartTime = currentTime;

    private boolean b;

    public boolean VISION_ENABLED = true;

    public void changeState(int i) {
        state = i;
        stateStartTime = currentTime;
    }

    public final Transform[] transforms = {new Transform(22.2, 16.5, 0),
            new Transform(22.2, 16.5, -0.462),
            new Transform(50, 20, 0),
            new Transform(62, 12, 0),
            new Transform(86, 36, 0),
            new Transform(120, 12, 0),
            SHOOTING_T,
            new Transform(82, SHOOTING_T.pos.y, 0)
    };

    /*
    public final Transform[] transforms = {STARTING_T[STARTING_T_I],
            STARTING_T[STARTING_T_I],
            STARTING_T[STARTING_T_I],
            STARTING_T[STARTING_T_I],
            STARTING_T[STARTING_T_I],
            STARTING_T[STARTING_T_I],
            STARTING_T[STARTING_T_I],
            STARTING_T[STARTING_T_I]
    };
     */

    public void moveState(int i) {
        moveState(i, state + 1);
    }

    public void moveState(int i, int nextState) {
        if(!SERVOS_ACTIVE) {
            changeState(nextState);
            return;
        }
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
    }

    @Override
    public void initialize() {
        super.initialize();
        resetPosition(0);
    }

    @Override
    public void tick() {
        wheelPowers = D;
        switch(state) {
            case 0:
                if(VISION_ENABLED) {
                    startVision();
                }
                moveState(0);
                break;
            case 1:
                moveState(1);
                break;
            case 2:
                if(VISION_ENABLED) {
                    if (cam.getFps() > 0.0) {
                        changeState(3);
                    }
                } else {
                    changeState(4);
                }
                break;
            case 3:
                if(VISION_ENABLED) {
                    telemetry.addData("Stack Size:", filter.stackSize);
                    telemetry.addData("Image Area", filter.hsvThresholdOutput().height() * filter.hsvThresholdOutput().width());
                    telemetry.addData("Unfiltered Contour Count:", filter.findContoursOutput().size());
                    telemetry.addData("Filtered Contour Count:", filter.filterContoursOutput().size());
                    telemetry.addData("LastRatio:", filter.lastRatio);
                    telemetry.addData("FPS:", cam.getFps());
                }
                if(currentTime - stateStartTime > 3000 && SERVOS_ACTIVE) { //|| stackSize != 0) {
                    changeState(4);
                }
                break;
            case 4:
                wobbleAimIndex = 1;
                moveState(2);
                break;
            case 5:
                wobbleAimIndex = 2;
                if(VISION_ENABLED) {
                    switch (filter.stackSize) {
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
                } else {
                    changeState(7);
                }
            case 7:
                moveState(3, 10);
                break;
            case 8:
                moveState(4, 10);
            case 9:
                moveState(5, 10);
            case 10:
                if(currentTime - stateStartTime > 1000) {
                    changeState(11);
                }
                break;
            case 11:
                wobbleHandIndex = 1;
                if(currentTime - stateStartTime > 1500) {
                    wobbleHandIndex = 0;
                    launchIndex = 1;
                    changeState(12);
                } else if(currentTime - stateStartTime > 500) {
                    wobbleAimIndex = 0;
                }
                break;
            case 12:
                moveState(6);
                break;
            case 13:
                if(currentTime > 27000) {
                    changeState(14);
                } else if(currentTime - stateStartTime > 4500) {
                    powerIntake(false, true, false);
                } else if(currentTime - stateStartTime > 700) {
                    wobbleHandIndex = 0;
                }
                break;
            case 14:
                moveState(7, 99);
                break;
            case 23:
                if(shotsFired < 3) {
                    powerIntake(false, true, false);
                    if(currentTime - stateStartTime > 1000) {
                        changeState(24);
                    }
                } else {
                    changeState(99);
                }
                break;
            case 24:
                powerIntake(false, true, true);
                if(currentTime - stateStartTime > 500) {
                    shotsFired++;
                    changeState(23);
                }
                break;
            case 99:
                break;
        }
        if(SERVOS_ACTIVE) {
            updateMotors();
            updateWobbleAim();
            updateWobbleHand();
            updateLaunchAim();
        }
        telemetry.addData("State:", state);
        updateTelemetry();
    }
}
