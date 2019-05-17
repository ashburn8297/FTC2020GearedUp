package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;


@TeleOp(name="Mecanum Test")
public class teleOp extends OpMode {
    robotBase robot       = new robotBase();
    ElapsedTime runtime   = new ElapsedTime();

    boolean gyroLockActive = true; //If the gyroLock system available?
    private double gyroLockPressed = 0.0; //How long ago was it pressed?
    boolean gyroLock = false; //Is a heading currently known?
    double lockHead = 0.0; //Set default heading to 0 deg

    private double headingResetPressed = 0.0; //How long ago was the heading reset?

    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);
        sleep(500);
        //Start the robot at zero power, using encoders, and float zero power
        robot.brake();
        robot.runUsingEncoders();
        robot.baseFloat();

        while (robot.modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("Calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
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
    public void loop(){

        //Maybe take in controller inputs and distribute via variables, not hardware calls


        //Add gyro lock if not turning and gyroLock is toggled on.
        if(Math.abs(gamepad1.right_stick_x)<.05 && gyroLockActive){
            if(gyroLock == false) { //See if new heading has been set. Should update once.
                //Find current heading
                lockHead = robot.gyroMR.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                //Let system know new heading has been located and not to refresh
                gyroLock = true;
            }
            //Run off this reset heading
            robot.mecanumGyroLock(gamepad1.left_stick_x, -gamepad1.left_stick_y, lockHead);
        }
        else {
            //If previous checks fail (either turning or gyroLock is disabled
            //Use normal operation
            robot.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            //Reset heading status to not found
            gyroLock = false;
        }

        //------------------------------------------------------------------------------------------
        //If x is pressed and has not been pressed in the last half second
        if(gamepad1.x && runtime.seconds() > gyroLockPressed){
            //Negate the current state of gyroLockActive (toggle)
            gyroLockActive = !gyroLockActive;

            //Set timer so that pause is reset to .5s
            gyroLockPressed = runtime.seconds() + .5;
        }

        //If y is pressed, reset Z axis heading and target heading
        if(gamepad1.y && runtime.seconds() > headingResetPressed){
            //Reset gyro and target heading
            robot.modernRoboticsI2cGyro.resetZAxisIntegrator();
            lockHead = 0.0;

            //Set timer so that pause is reset to .5s
            headingResetPressed = runtime.seconds() +.5;
        }

        //------------------------------------------------------------------------------------------
        telemetry.addData("Time", Math.round(runtime.seconds()));
        telemetry.addData("GyroLock", gyroLockActive ? "Online" : "Offline");
        telemetry.addData("Heading", robot.gyroMR.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Target", lockHead);
        telemetry.addLine("Left Joystick |")
                .addData(" x","%.2f", gamepad1.left_stick_x)
                .addData("y","%.2f", -gamepad1.left_stick_y);
        telemetry.addLine("Right Joystick |")
                .addData(" x","%.2f", gamepad1.right_stick_x);
        telemetry.addLine("Front Motor Powers")
                .addData(" FL","%.2f", robot.FLD.getPower())
                .addData("FR","%.2f", robot.FRD.getPower());
        telemetry.addLine("Rear Motor Powers")
                .addData(" RL","%.2f", robot.RLD.getPower())
                .addData("RR","%.2f", robot.RRD.getPower());
        telemetry.update();
    }

    @Override
    public void stop(){
        telemetry.addData("Stopped after", runtime.seconds());
        telemetry.update();
    }
}
