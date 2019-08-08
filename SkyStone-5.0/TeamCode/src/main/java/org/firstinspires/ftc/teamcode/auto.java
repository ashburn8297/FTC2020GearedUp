package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Gyro Test")
public class auto extends LinearOpMode {
    robotBase robot = new robotBase();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, true);
        sleep(500);

        //Start the robot at zero power, using encoders, and float zero power
        robot.brake();
        robot.runUsingEncoders();
        robot.baseFloat();

        //calibrate gyro
        runtime.reset();

        while (!isStopRequested() && robot.navxMicro.isCalibrating() && opModeIsActive())  {
            telemetry.addData("Calibrating NavX", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        //------------------------------------------------------------------------------------------
        runtime.reset();

        /*robot.turn(30, 1, 5.0, true, auto.this, telemetry);
        sleep(500);
        robot.turn(60, 1, 5.0, true, auto.this, telemetry);
        sleep(500);
        robot.turn(0, 1, 5.0, true, auto.this, telemetry);
        sleep(500);
        robot.turn(90, 1, 5.0, true, auto.this, telemetry);
        sleep(500);
        robot.turn(0, 1, 5.0, true, auto.this, telemetry);
        sleep(500);*/

        robot.countDegrees(auto.this, telemetry);

        stop();

    }
}
