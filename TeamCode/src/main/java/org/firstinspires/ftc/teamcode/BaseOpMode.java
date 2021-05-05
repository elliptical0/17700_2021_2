package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

    ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double currentTime;
    double deltaTime;

    DcMotor[] drive = new DcMotor[4];
    DcMotor[] motors = new DcMotor[3];
    DcMotor intake;
    DcMotor flywheel;
    double lastFlywheelPos;
    int flywheelAtSpeedTicks = 0;
    CRServo magazine;
    Servo wobbleAim;
    int wobbleAimIndex = 0;
    Servo wobbleHand;
    int wobbleHandIndex = 0;
    Servo counterweight;
    Servo[] launchAim = new Servo[2]; //Must be in unison
    int launchIndex = 0;

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
        clock.reset();
    }

    public void tick() {
        updateTelemetry();
    }

    public void updateTime() {
        deltaTime = clock.time() > currentTime ? clock.time() - currentTime : clock.time();
        currentTime = clock.time();
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
        telemetry.addData("", "");
        telemetry.addData("To calibrate", "drag the robot forward " + CALIB_DIST + " inches (and do nothing else) and read the value below.");
        telemetry.addData("DEADWHEEL_RADIUS (" + CALIB_DIST + ")", CALIB_DIST / (2 * Math.PI * ((encoderPos[0] + encoderPos[1]) / 2) / TICKS_PER_REV));
        telemetry.addData("To calibrate", "spin the robot in place counterclockwise " + CALIB_SPINS + " times (and do nothing else) and read the values below. (Depends upon DEADWHEEL_RADIUS)");
        telemetry.addData("LATERAL_DISTANCE (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[0]) - encoderToInch(encoderPos[1])) / (2 * CALIB_SPINS * Math.PI));
        telemetry.addData("REAR_OFFSET (" + CALIB_SPINS + ")", (encoderToInch(encoderPos[2]) / (2 * CALIB_SPINS * Math.PI)));
        telemetry.update();
    }

    public void powerIntake(boolean in, boolean fly, boolean mag) {
        if(in) {
            intake.setPower(1);
            magazine.setPower(-1);
            flywheel.setPower(1);
        } else if(FLYWHEEL_ENCODER) {
            if(fly) {
                flywheel.setPower(-1);
                if (calculateRPM(Math.abs(flywheel.getCurrentPosition() - lastFlywheelPos), deltaTime) > FLYWHEEL_MAX_RPM * 0.95) {
                    magazine.setPower(1);
                } else if(mag) {
                    magazine.setPower(1);
                } else {
                    magazine.setPower(0);
                }
                intake.setPower(0);
            }
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
            launchAim[i].setPosition(LAUNCH_AIM_POSITIONS[launchIndex]);
        }
        counterweight.setPosition(COUNTERWEIGHT_POSITIONS[launchIndex]);
    }

    public boolean servoAtPos(Servo servo, double pos) {
        return Math.abs(servo.getController().getServoPosition(0) - pos) < DEADZONE_SERVO;
    }

    public void resetPosition() {
        for(i = 0; i < 3; i++) {
            encoderResetPos[i] = (drive[i].getCurrentPosition());
        }
        transform.set(0, 0, 0);
    }

    public void startVision(final boolean autoMode) {
        if(autoMode) {
            filter = new ImgFilter();
        }
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cam.setPipeline(filter);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                resumeVision();
                if(autoMode) {
                    cam.setPipeline(filter);
                }
            }
        });
    }

    public void pauseVision() {
        cam.stopStreaming();
    }

    public void resumeVision() {
        cam.openCameraDevice();
        cam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    public void initialize() {
        transform = SharedVariables.transform;
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
