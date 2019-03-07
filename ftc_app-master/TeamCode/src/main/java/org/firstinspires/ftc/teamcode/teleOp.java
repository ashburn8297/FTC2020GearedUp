package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Mecanum Test")
public class teleOp extends OpMode {
    robotBase robot       = new robotBase();
    ElapsedTime runtime   = new ElapsedTime();

    boolean gyroLock = false;
    double lockHead = 0.0;

    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Start the robot at zero power, using encoders, and float zero power
        robot.brake();
        robot.runUsingEncoders();
        robot.baseFloat();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop(){


        /* This test TeleOp aims to accomplish the following:
         * - Allow both normal operation and gyro-locked heading control
         *
         * It does this by:
         * Monitoring the turn axis, and if the input is less than .05 (deadzone), the robot should lock heading and maintain it
         * But, if the turn axis is greater than .05, the robot reverts to normal operation and accepts all three inputs without a gyro lock.
         */

        //Add gyro lock if not turning.
        if(Math.abs(gamepad2.right_stick_x)<.05){ //If turning is not happening
            if(gyroLock == false) { //See if new heading has been set
                lockHead = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                gyroLock = true;
            }
            robot.mecanumGyroLock(gamepad1.left_stick_x, -gamepad1.left_stick_y, lockHead);
        }
        else {
            robot.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad2.right_stick_x);
            gyroLock = false;
        }

        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("Controller X", gamepad1.left_stick_x);
        telemetry.addData("Controller Y", gamepad1.left_stick_y);
        telemetry.addData("FL", robot.FLD.getPower());
        telemetry.addData("FR", robot.FRD.getPower());
        telemetry.addData("RL", robot.RLD.getPower());
        telemetry.addData("RR", robot.RRD.getPower());
        telemetry.update();
    }

    @Override
    public void stop(){
    }
}
