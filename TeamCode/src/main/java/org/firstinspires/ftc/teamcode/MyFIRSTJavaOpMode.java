package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@Deprecated
@Disabled
@TeleOp(name="MyFIRSTJavaOpMode", group="default")
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private Gamepad user1;
    @Override
    public void runOpMode() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0"); //Front-Left, Negative is Forward
        motor1 = hardwareMap.get(DcMotor.class, "motor1"); //Front-Right, Positive is Forward
        motor2 = hardwareMap.get(DcMotor.class, "motor2"); //Back-Right, Positive is Forward
        motor3 = hardwareMap.get(DcMotor.class, "motor3"); //Back-Left, Negative is Forward
        user1 = this.gamepad1;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double[] wheelPowers;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            wheelPowers = MyMathLib.mecMath(user1.left_stick_x, user1.left_stick_y, (user1.left_trigger - user1.right_trigger));
            motor0.setPower(wheelPowers[0]);
            motor1.setPower(wheelPowers[1]);
            motor2.setPower(wheelPowers[2]);
            motor3.setPower(wheelPowers[3]);

            /* Used this to identify motor locations.
            if(user1.a) {
                motor0.setPower(0.25);
            }
            if(user1.b) {
                motor1.setPower(0.25);
            }
            if(user1.x) {
                motor2.setPower(0.25);
            }
            if(user1.y) {
                motor3.setPower(0.25);
            }
             */

            telemetry.addData("Controller Input", user1.left_stick_x + ", " + user1.left_stick_y + ", " + (user1.right_trigger - user1.left_trigger));
            telemetry.addData("Motor Output", wheelPowers[0] + ", " + wheelPowers[1] + ", " + wheelPowers[2] + ", " + wheelPowers[3]);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
