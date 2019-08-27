package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name = "Test")
public class auto extends LinearOpMode {
    robotBase robot = new robotBase();
    ElapsedTime runtime = new ElapsedTime();

    public static double distnaceX = 0;
    public static double distanceY = 40;
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

        robot.odometry(auto.this, telemetry, distanceY, 30.0);

        stop();

    }
}
