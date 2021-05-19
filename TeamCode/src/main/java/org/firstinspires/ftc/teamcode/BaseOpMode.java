package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * The base op mode, contains useful variables and debug stuff which is useful in auto and teleop.
 * @author TM
 */
public class BaseOpMode extends LinearOpMode {
    int i;
    double d;

    double startTime;
    double currentTime;
    double deltaTime;

    DcMotor[] drive = new DcMotor[4];
    DcMotor[] motors = new DcMotor[3];
    DcMotor intake;
    DcMotor flywheel;
    double lastFlywheelPosition;
    double flywheelRPM;
    double[] flywheelRPMHistory = new double[30];
    int flywheelRPMHistoryIndex = 0;
    CRServo magazine;
    Servo wobbleAim;
    int wobbleAimIndex = 0;
    Servo wobbleHand;
    int wobbleHandIndex = 0;
    Servo counterweight;
    Servo[] launchAim = new Servo[2]; //Must be in unison
    int launchIndex = 0;

    ColorSensor[] colorSensors = new ColorSensor[2];

    int stackSize = 0; //0: NULL  1: ZERO  2: ONE  3: FOUR
    int[] colorSensorHistory = new int[25];
    int colorSensorHistoryIndex = 0;

    ImgFilter filter;
    ArrayList<MatOfPoint> rings;
    OpenCvCamera cam;

    double[] wheelPowers;

    double[] encoderPos = new double[3];
    double[] encoderResetPos = new double[3];
    double[] encoderDelta = new double[3];
    Transform transform = new Transform(0, 0, 0);

    int shoti = 0;

    /**
     * Called once, after waitForStart() and before tick().
     */
    public void beginTick() {
        startTime = System.currentTimeMillis();
    }

    public void tick() {
        updateTelemetry();
    }

    public void updateTime() {
        deltaTime = System.currentTimeMillis() - startTime - currentTime;
        currentTime = System.currentTimeMillis() - startTime;
    }

    public void updateOdometry() {
        for(i = 0; i < 3; i++) {
            d = (drive[i].getCurrentPosition() - encoderResetPos[i]) * ENCODER_DIRECTIONS[i];
            encoderDelta[i] = d - encoderPos[i];
            encoderPos[i] = d;
        }
        transform = updatePosition(transform, encoderDelta, encoderPos);
    }

    public void updateTelemetry() {
        telemetry.addData("Transform", transform);
        telemetry.addData("Build Name:", BUILD_NAME);
        telemetry.addData("DeltaTime:", deltaTime);
        if(SERVOS_ACTIVE) { telemetry.addData("FlywheelRPM:", flywheelRPM); }
        if(COLOR_ACTIVE) {
            telemetry.addData("ColorSensor0", colorSensors[0].red() + ", " + colorSensors[0].green() + ", " + colorSensors[0].blue());
            telemetry.addData("ColorSensor0 Red:Blue Ratio", (double) colorSensors[0].red() / colorSensors[0].blue());
            telemetry.addData("ColorSensor1", colorSensors[1].red() + ", " + colorSensors[1].green() + ", " + colorSensors[1].blue());
        }
        telemetry.addData("", "");
        telemetry.addData("To calibrate", "drag the robot forward " + CALIB_DIST + " inches (and do nothing else) and read the value below.");
        telemetry.addData("DEADWHEEL_RADIUS (" + CALIB_DIST + ")", CALIB_DIST / (2 * Math.PI * ((encoderPos[0] + encoderPos[1]) / 2) / TICKS_PER_REV));
        telemetry.addData("To calibrate", "spin the robot in place counterclockwise " + CALIB_SPINS + " times (and do nothing else) and read the values below. (Depends upon DEADWHEEL_RADIUS)");
        telemetry.addData("LATERAL_DISTANCE (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[0]) - encoderToInch(encoderPos[1])) / (2 * CALIB_SPINS * Math.PI));
        telemetry.addData("REAR_OFFSET (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[2]) / (2 * CALIB_SPINS * Math.PI)));
        telemetry.update();
    }

    public void flywheelRPM() {
        flywheelRPM = calculateRPM(Math.abs(flywheel.getCurrentPosition() - lastFlywheelPosition), deltaTime);
        flywheelRPMHistory[flywheelRPMHistoryIndex] = flywheelRPM;
        flywheelRPMHistoryIndex = (flywheelRPMHistoryIndex + 1) % flywheelRPMHistory.length;
        flywheelRPM = 0;
        for(double d : flywheelRPMHistory) {
            flywheelRPM += d;
        }
        flywheelRPM /= flywheelRPMHistory.length;
    }

    public void powerIntake(boolean in, boolean fly, boolean mag) {
        flywheelRPM();
        if(in) {
            intake.setPower(1);
            magazine.setPower(-1);
            flywheel.setPower(1);
        } else if(FLYWHEEL_ENCODER) {
            if(fly) {
                flywheel.setPower(-1);
                if (flywheelRPM > FLYWHEEL_MAX_RPM * 0.95) {
                    magazine.setPower(1);
                } else if(mag) {
                    magazine.setPower(1);
                } else {
                    magazine.setPower(0);
                }
            } else {
                flywheel.setPower(0);
                magazine.setPower(0);
            }
            intake.setPower(0);
        } else {
            //Flywheel Controls
            if(fly) {
                flywheel.setPower(-1);
            } else {
                flywheel.setPower(0);
            }
            //Magazine Controls
            if(mag) {
                magazine.setPower(1);
            } else {
                magazine.setPower(0);
            }
            intake.setPower(0);
        }
        lastFlywheelPosition = flywheel.getCurrentPosition();
    }

    public void updateMotors() {
        for (i = 0; i < 4; i++) {
            drive[i].setPower(wheelPowers[i] * MOTORCALIB[i]);
        }
    }

    public void updateWobbleAim() {
        wobbleAim.setPosition(WOBBLE_AIM_POSITIONS[wobbleAimIndex]);
    }

    public void updateWobbleHand() {
        wobbleHand.setPosition(WOBBLE_HAND_POSITIONS[wobbleHandIndex]);
    }

    public void updateLaunchAim() {
        for(i = 0; i < 2; i++) {
            launchAim[i].setPosition(LAUNCH_AIM_POSITIONS[launchIndex] + LAUNCH_ADJUST[i]);
        }
        counterweight.setPosition(COUNTERWEIGHT_POSITIONS[launchIndex]);
    }

    public void readColorSensors() {
        if(colorSensorHistoryIndex < colorSensorHistory.length) {
            boolean[] detected = {false, false};

            /*
            int[] color;
            outer: for(i = 0; i < 2; i++) {
                color = new int[]{colorSensors[i].red(), colorSensors[i].green(), colorSensors[i].blue()};
                for(int n = 0; n < 3; n++) {
                    if(color[n] < RCOLOR[0][n] || color[n] > RCOLOR[1][n]) {
                        continue outer;
                    }
                }
                detected[i] = true;
            }
             */
            for(i = 0; i < 2; i++) {
                detected[i] = ((double) colorSensors[i].red() / colorSensors[i].blue()) > RED_TO_BLUE_MIN_RATIO;
            }

            if(detected[0] && detected[1]) {
                colorSensorHistory[colorSensorHistoryIndex] = 3;
            } else if(detected[0] || detected[1]) {
                colorSensorHistory[colorSensorHistoryIndex] = 2;
            } else {
                colorSensorHistory[colorSensorHistoryIndex] = 1;
            }
            colorSensorHistoryIndex++;
        }
        if(colorSensorHistoryIndex >= colorSensorHistory.length) {
            int[] tally = new int[3];
            for(int n : colorSensorHistory) {
                if(n > 0) {
                    tally[n - 1] = tally[n - 1] + 1;
                }
            }
            int highest = 0;
            for(i = 1; i < tally.length; i++) {
                if(tally[i] > tally[highest]) {
                    highest = i;
                }
            }
            stackSize = highest + 1;
        }
    }

    @Deprecated
    public boolean servoAtPos(Servo servo, double pos) {
        telemetry.addData("ServoPosition: ", servo.getController().getServoPosition(0));
        return Math.abs(servo.getController().getServoPosition(0) - pos) < DEADZONE_SERVO;
    }

    public void resetPosition(int index) {
        for(i = 0; i < 3; i++) {
            encoderResetPos[i] = (drive[i].getCurrentPosition());
        }
        transform = STARTING_T[index];
    }

    public void startVision() {
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        filter = new ImgFilter();
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                resumeVision();
                cam.setPipeline(filter);
            }
        });
    }

    public void pauseVision() {
        cam.stopStreaming();
    }

    public void resumeVision() {
        cam.openCameraDevice();
        cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    public void initialize() {
        for (i = 0; i < 4; i++) {
            drive[i] = hardwareMap.get(DcMotor.class, "motor" + i);
        }
        if(SERVOS_ACTIVE) {
            intake = hardwareMap.get(DcMotor.class, "motor5");
            flywheel = hardwareMap.get(DcMotor.class, "motor6");
            counterweight = hardwareMap.get(Servo.class, "servo6");
            counterweight.setDirection(Servo.Direction.REVERSE);
            magazine = hardwareMap.get(CRServo.class, "servo5");
            wobbleAim = hardwareMap.get(Servo.class, "servo4");
            wobbleHand = hardwareMap.get(Servo.class, "servo3");
            for (i = 0; i < 2; i++) {
                launchAim[i] = hardwareMap.get(Servo.class, "servo" + i);
                launchAim[i].setDirection(i == 0 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            }
        }
        if(COLOR_ACTIVE) {
            for (i = 0; i < 2; i++) {
                colorSensors[i] = hardwareMap.get(ColorSensor.class, "color" + i);
            }
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        beginTick();
        while(opModeIsActive()) {
            updateTime();
            updateOdometry();
            tick();
        }
    }
}
