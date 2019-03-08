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

    boolean gyroLockActive = true; //If the gyroLock system available?
    private double gyroLockPressed = 0.0; //How long ago was it pressed?
    boolean gyroLock = false; //Is a heading currently known?
    double lockHead = 0.0; //Set default heading to 0 deg

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


        /**
         * @TODO Confirm gyro still works after transition to teleOp
         * @TODO Check over mecanum code
         * @TODO See if auto.this works for autonomous instructions
         */


        /* This test TeleOp aims to accomplish the following:
         * - Allow both normal operation and gyro-locked heading control
         *
         * It does this by:
         * Monitoring the turn axis, and if the input is less than .05 (deadzone) and the system is online (gyroLockActive), the robot should lock heading and maintain it
         * But, if the turn axis is greater than .05 or the gyroLock is toggled off, the robot reverts to normal operation and accepts all three inputs without a gyro lock.
         */

        //Add gyro lock if not turning and gyroLock is toggled on.
        if(Math.abs(gamepad2.right_stick_x)<.05 && gyroLockActive){
            if(gyroLock == false) { //See if new heading has been set. Should update once.
                //Find current heading
                lockHead = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                //Let system know new hading has been located and not to refresh
                gyroLock = true;
            }
            //Run off this reset heading
            robot.mecanumGyroLock(gamepad1.left_stick_x, -gamepad1.left_stick_y, lockHead);
        }
        else {
            //If previous checks fail (either turning or gyroLock is disabled
            //Use normal operation
            robot.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad2.right_stick_x);
            //Reset heading status to not found
            gyroLock = false;
        }

        //If x is pressed and has not been pressed in the last half second
        if(gamepad1.x && runtime.seconds() > gyroLockPressed){
            //Negate the current state of gyroLockActive (toggle)
            gyroLockActive = !gyroLockActive;
            //Set timer so that pause is reset to .5s
            gyroLockPressed = runtime.seconds() + .5;
        }


        telemetry.addData("Time", runtime.seconds());
        telemetry.addData("GyroLock", gyroLockActive ? "Available" : "Offline");
        telemetry.addData("Controller X", gamepad1.left_stick_x);
        telemetry.addData("Controller Y", -gamepad1.left_stick_y);
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
