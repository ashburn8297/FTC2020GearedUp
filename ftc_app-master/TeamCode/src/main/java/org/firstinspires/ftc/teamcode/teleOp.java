package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Mecanum Test")
public class teleOp extends OpMode {
    robotBase robot = new robotBase();
    ElapsedTime runtime = new ElapsedTime();

    private double headingResetPressed = 0.0; //How long ago was the heading reset?

    public void init() {
        /*
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);
        sleep(500);
        //Start the robot at zero power, using encoders, and float zero power
        robot.brake();
        robot.runUsingEncoders();
        robot.baseBrake();

        while (robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("Calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        robot.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        //------------------------------------------------------------------------------------------
        telemetry.addData("Time", Math.round(runtime.seconds()));
        telemetry.addLine("Left Joystick |")
                .addData(" x", "%.2f", gamepad1.left_stick_x)
                .addData("y", "%.2f", -gamepad1.left_stick_y);
        telemetry.addLine("Right Joystick |")
                .addData(" x", "%.2f", gamepad1.right_stick_x);
        telemetry.addLine("Front Motor Powers")
                .addData(" FL", "%.2f", robot.FLD.getPower())
                .addData("FR", "%.2f", robot.FRD.getPower());
        telemetry.addLine("Rear Motor Powers")
                .addData(" RL", "%.2f", robot.RLD.getPower())
                .addData("RR", "%.2f", robot.RRD.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Stopped after", runtime.seconds());
        telemetry.update();
    }
}
