package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static android.os.SystemClock.sleep;


public class robotBase {

    public static final double HEADING_THRESHOLD = .25;
    public static final String VUFORIA_KEY = "AbEDH9P/////AAABmcFPgUDLz0tMh55QD8t9w6Bqxt3h/G+JEMdItgpjoR+S1FFRIeF/w2z5K7r/nUzRZKleksLHPglkfMKX0NltxxpVUpXqj+w6sGvedaNq449JZbEQxaYe4SU+3NNi0LBN879h9LZW9RxJFOMt7HfgssnBdg+3IsiwVKKYnovU+99oz3gJkcOtYhUS9ku3s0Wz2n6pOu3znT3bICiR0/480N63FS7d6Mk6sqN7mNyxVcRf8D5mqIMKVNGAjni9nSYensl8GAJWS1vYfZ5aQhXKs9BPM6mST5qf58Tg4xWoHltcyPp0x33tgQHBbcel0M9pYe/7ub1pmzvxeBqVgcztmzC7uHnosDO3/2MAMah8qijd";
    /*Public hardware members*/
    public DcMotor FLD = null; //Front Left Drive Motor, "FLD"
    public DcMotor FRD = null; //Front Right Drive Motor, "FRD"
    public DcMotor RLD = null; //Rear Left Drive Motor, "RLD"
    public DcMotor RRD = null; //Rear Right Drive Motor, "RRD"
    public double prev_heading = 0;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    IntegratingGyroscope gyroNV;                //For polymorphism
    NavxMicroNavigationSensor navxMicro;        //To initialize gyroscope, "navx" on phones
    AnalogInput odometryL; //"odometryL"
    AnalogInput odometryR; //"odometryR"
    AnalogInput odometryT; //"odometryT"
    /*Local opMode members*/
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    //Empty constructor for object creation.
    public robotBase() {

    }

    /**
     * Scale input to a modified sigmoid curve
     *
     * @param power is a double input from the mecanum method
     * @return modified power value per the sigmoid curve
     * graph here -> https://www.desmos.com/calculator/7ehynwbhn6
     */
    public static double powerScale(double power) {
        //If power is negative, log this and save for later
        //If positive, keep neg to 1, so that num is always positive
        //If negative, change to -1, so that ending number is negative
        double neg = 1;
        if (power < 0)
            neg = -1;

        //Set power to always be positive
        power = Math.abs(power);

        //modify to sigmoid
        //delivers 0 power at 0, and 1 power at 1, varies in between
        power = (.8 / (1 + Math.pow(Math.E, -12 * (power - .5))));
        //graph here -> https://www.desmos.com/calculator/tpdxkzhned


        //Make sure value falls between -1 and 1

        //Explaining the math;
        //Take the maximum of -1 and the power * neg value (makes sure it's greater than -1
        //Then take the minimum of 1 and the power * neg value (makes sure it's less than 1)
        //Multiply the result by 100
        //Then divide by 100 and get the floating point answer (rounds to 2 decimal places)
        return Math.round(Math.min(Math.max(power * neg, -1), 1) * 100) / 100.0;
    }

    public static boolean isBetween(double LB, double val, double UB) {
        return LB < val && val < UB;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry t, boolean gyroOn) {
        hwMap = ahwMap;

        //Initialize the four drive base motors
        FLD = hwMap.get(DcMotor.class, "FLD");
        FRD = hwMap.get(DcMotor.class, "FRD");
        RLD = hwMap.get(DcMotor.class, "RLD");
        RRD = hwMap.get(DcMotor.class, "RRD");

        //Set the drive base motors so that +1 drives forward
        /*It's possible these values are reversed*/
        FLD.setDirection(DcMotor.Direction.REVERSE);
        FRD.setDirection(DcMotor.Direction.FORWARD);
        RLD.setDirection(DcMotor.Direction.REVERSE);
        RRD.setDirection(DcMotor.Direction.FORWARD);
        //Stop all motors when robot is initialized
        brake();

        //Set all motors so that they begin without encoders enabled
        runWithoutEncoders();

        //Set all motors so when zero power is issued, motors to not actively resist (brake)
        baseFloat();

        /**@Important
         * Be sure to Factory reset occasionally
         * Link -> https://pdocs.kauailabs.com/navx-micro/wp-content/uploads/2019/02/navx-micro_robotics_navigation_sensor_user_guide.pdf Page 35
         * https://pdocs.kauailabs.com/navx-micro/guidance/gyroaccelerometer-calibration/
         * */

        odometryL = hwMap.get(AnalogInput.class, "odometryL");
        odometryR = hwMap.get(AnalogInput.class, "odometryR");


        if (gyroOn) {


            //Configure the NavXMicro for use
            navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx"); //Used for auto
            gyroNV = navxMicro;
            t.addData("gyroNV", "ONLINE");
            t.update();
        }
    }

    //Run the robot, ignoring encoder values
    public void runWithoutEncoders() {
        FLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Run the robot using encoder values as velocity check
    public void runUsingEncoders() {
        FLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Run the robot using encoders to run to a specified position (encoder ticks);
    public void runToPosition() {
        FLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders() {
        FLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Set all motors to float when zero power command is issued
    public void baseFloat() {
        FLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Set all motors to actively resist when zero power command is issued
    public void baseBrake() {
        FLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Set all motors to zero power
    public void brake() {
        FLD.setPower(0);
        FRD.setPower(0);
        RLD.setPower(0);
        RRD.setPower(0);
    }

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward, double Turn, boolean useCurve) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //Ramp these values with powerScale's values.
        if (useCurve == true) {
            FLD.setPower(powerScale(v1));
            FRD.setPower(powerScale(v2));
            RLD.setPower(powerScale(v3));
            RRD.setPower(powerScale(v4));
        } else {
            FLD.setPower(v1);
            FRD.setPower(v2);
            RLD.setPower(v3);
            RRD.setPower(v4);
        }
    }

    /**
     * A method to drive to a specific position via a desired (X,Y) pair given in inches.
     *
     * @param xInches desired X axis translation (left right)
     * @param yInches desired Z axis translation (front back)
     * @param timeout maximum amount of time for the action to occur
     * @param opMode  the current state of the opMode
     * @param t       the current opMode's telemetry object, not opMode param
     * @TODO Design this system using gyro and dead wheels
     */
    public void translate(double xInches, double yInches, double timeout, LinearOpMode opMode, Telemetry t) {
        //Create a local timer
        resetEncoders();
        runUsingEncoders();
        baseBrake();

        period.reset(); //Start the clock

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        //Master speed modifier
        double scale = .5;

        //Master distance constants
        final double inches_per_rotation = 6.28;
        final double volts_per_degree = 3.3 / 360;
        final double high_speed_motor = .5;
        final double low_speed_motor = .2;

        boolean ableToRegisterRevX = false;
        boolean ableToRegisterRevY = false;


        //Generate rotational deltas
        double degreesX = (xInches / inches_per_rotation) * 360;
        int full_rotationsX = (int) (degreesX / 360); //in full revolutions
        int rotationsX = 0;
        int partial_rotationsX = (int) (degreesX % 360); //in degrees
        boolean doneX = false;

        //---------------------
        double degreesY = (yInches / inches_per_rotation) * 360;
        int full_rotationsY = (int) (degreesY / 360); //in full revolutions
        int rotationsY = 0;
        int partial_rotationsY = (int) (degreesY % 360); //in degrees
        boolean doneY = false;


        //Voltage Math
        double starting_voltageX = Math.floor(odometryT.getVoltage() * 1000) / 1000;
        if (partial_rotationsX < 0) {
            rotationsX++;
        }
        double current_voltageT = 0.0;
        double ending_voltageX = Math.floor(((starting_voltageX - (Math.abs(partial_rotationsX) * (volts_per_degree))) % 3.3) * 1000) / 1000;
        if (ending_voltageX < 0) {
            ending_voltageX = ending_voltageX + 3.3;
        }

        //-------------------
        double starting_voltageY = Math.floor(odometryL.getVoltage() * 1000) / 1000;
        if (partial_rotationsY < 0) {
            rotationsY++;
        }
        double current_voltageL = 0.0;
        double ending_voltageY = Math.floor(((starting_voltageY - (Math.abs(partial_rotationsY) * (volts_per_degree))) % 3.3) * 1000) / 1000;
        if (ending_voltageY < 0) {
            ending_voltageY = ending_voltageY + 3.3;
        }


        //Calculate direction
        double dirX = 0;
        if (xInches > 0) {
            dirX = 1;
        } else {
            dirX = -1;
        }

        //------------------------------
        double dirY = 0;
        if (yInches > 0) {
            dirY = 1;
        } else {
            dirY = -1;
        }


        //Vector calculation
        double theta = Math.atan2(yInches, xInches) - Math.PI / 4;

        //Front Left and Right Rear
        double v1 = Math.cos(theta) * scale;
        //Front Right and Rear Left
        double v2 = -Math.sin(theta) * scale;

        double velX = 0.0;
        double velY = 0.0;

        while (opMode.opModeIsActive() && Math.abs(rotationsX) < Math.abs(full_rotationsX)
                && Math.abs(rotationsY) < Math.abs(full_rotationsY) && period.seconds() < timeout) {

            current_voltageL = Math.floor(odometryL.getVoltage() * 1000) / 1000;
            current_voltageT = Math.floor(odometryL.getVoltage() * 1000) / 1000;

            if (Math.abs(rotationsX) < Math.abs(full_rotationsX))
                velX = v1;
            else
                velX = 0;

            if (Math.abs(rotationsX) < Math.abs(full_rotationsX))
                velY = v1;
            else
                velY = 0;

            //-------------------------------------
            if (isBetween(starting_voltageX - .15, current_voltageT, starting_voltageX + .15) && ableToRegisterRevX == true) {
                rotationsX++;
                ableToRegisterRevX = false;
            }

            //has robot moved enough to register revolution
            if (ableToRegisterRevX == false && !isBetween(starting_voltageX - .2, current_voltageT, starting_voltageX + .2)) {
                ableToRegisterRevX = true;
            }

            //---------------------------------------
            if (isBetween(starting_voltageY - .15, current_voltageL, starting_voltageY + .15) && ableToRegisterRevY == true) {
                rotationsY++;
                ableToRegisterRevY = false;
            }

            //has robot moved enough to register revolution
            if (ableToRegisterRevY == false && !isBetween(starting_voltageY - .2, current_voltageL, starting_voltageY + .2)) {
                ableToRegisterRevY = true;
            }

            mecanum(velX * dirX, velY * dirY, 0, false);

            packet.put("X Voltage", current_voltageT);
            packet.put("Y voltage", current_voltageL);
            dashboard.sendTelemetryPacket(packet);
            opMode.idle();
        }

        //-----------------------------------------------------
        while (opMode.opModeIsActive() && !doneX
                && !doneY && period.seconds() < timeout) {

            mecanum(velX * dirX * .5, velY * dirY * .5, 0, false);

            current_voltageL = Math.floor(odometryL.getVoltage() * 1000) / 1000;
            current_voltageT = Math.floor(odometryL.getVoltage() * 1000) / 1000;

            if (isBetween(ending_voltageX - .05, current_voltageT, ending_voltageX + .05)) {
                doneX = true;
                velX = 0.0;
            }
            if (isBetween(ending_voltageY - .05, current_voltageL, ending_voltageY + .05)) {
                doneY = true;
                velY = 0.0;
            }

            opMode.idle();
        }

        baseBrake();

        brake();
        sleep(250);
    }

    /**
     * Turn towards a given heading (deg)
     *
     * @param targetAngle desire heading post-turn
     * @param coeff       how quickly to turn about the point
     * @param timeout     maximum amount of time for the action to occur
     * @param fullStop    should the robot stop after executing the movement?
     * @param opMode      the current state of the opMode
     * @param t           the current opMode's telemetry object, not opMode param
     */
    public void turn(double targetAngle, double coeff, double timeout, boolean fullStop, LinearOpMode opMode, Telemetry t) {
        //Create a local timer
        period.reset();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        boolean found = false;

        double cycleHeading;
        double deltaHeading;
        double steer;
        double error;
        double neg = 1;

        runUsingEncoders();

        deltaHeading = targetAngle - prev_heading;

        if (deltaHeading < -180)
            deltaHeading += 360;
        else if (deltaHeading >= 180)
            deltaHeading -= 360;
        //deltaHeading is now the relative turning vector


        //While active and within timeout
        while (opMode.opModeIsActive() && (period.seconds() < timeout) && (found == false)) {

            cycleHeading = gyroNV.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - prev_heading;

            double c = Math.abs(cycleHeading);
            double d = Math.abs(deltaHeading);

            if (deltaHeading < cycleHeading) {
                neg = 1;
            } else {
                neg = -1;
            }

            //algorithm here  @ https://www.desmos.com/calculator/thgycpx0n9
            double pct = 1 + ((c - d) / d);

            if (pct < 1) {
                error = .55 * Math.pow(2, (-pct));
            } else {
                error = .55 * Math.pow(2, (pct - 2));
            }
            //Modify speed with variable 'speed'
            steer = error * coeff * neg;

            mecanum(0.0, 0.0, steer, false);

            //Terminating condition
            if (Math.abs(cycleHeading - deltaHeading) < HEADING_THRESHOLD) {
                baseBrake();
                found = true;
                if (fullStop == true) {
                    brake();
                }
            }

            packet.put("Scaled Angle", cycleHeading);
            packet.put("Power", error * 100 * neg);
            packet.put("Angle", gyroNV.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            dashboard.sendTelemetryPacket(packet);

            opMode.idle();
        }
        prev_heading = gyroNV.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (fullStop == true) {
            brake();
            sleep(250);
        }
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return Math.round(result * 100) / 100;
    }

    public void odometry(LinearOpMode opMode, Telemetry t, double distance, double runtime) {

        period.reset();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        double volts_per_degree = 3.3 / 360;
        double high_speed_motor = .45;
        double low_speed_motor = .3;
        final double inches_per_rotation = 6.28;

        boolean ableToRegisterRev = false;

        //2.2.
        double degrees = (distance / inches_per_rotation) * 360;

        //2.3.2., 2.3.5.
        int full_rotations = (int) (degrees / 360); //in full revolutions
        int rotations = 0;

        int partial_rotations = (int) (degrees % 360); //in degrees
        boolean done = false;

        //2.4.2., rounded to 3 decimal places
        double starting_voltage = Math.floor(odometryL.getVoltage() * 1000) / 1000;

        //2.5.2., rounded to 3 decimal places

        if (partial_rotations < 0) {
            rotations++;
        }

        //Generate ending voltage by taking the starting voltage and subtracting the partial voltage delta and taking the remainder over 3.3
        double ending_voltage = Math.floor(((starting_voltage - (Math.abs(partial_rotations) * (volts_per_degree))) % 3.3) * 1000) / 1000;

        if (ending_voltage < 0) {
            ending_voltage = ending_voltage + 3.3;
        }

        //3.1.2, 3.2.2.
        double current_voltageL = 0.0;

        //2.6.
        double dir = 0;
        if (distance > 0) {
            dir = 1;
        } else {
            dir = -1;
        }

        //3.1.
        while (opMode.opModeIsActive() && Math.abs(rotations) < Math.abs(full_rotations) && period.seconds() < runtime) {
            mecanum(0, high_speed_motor * dir, 0, false);

            current_voltageL = Math.floor(odometryL.getVoltage() * 1000) / 1000;

            if (isBetween(starting_voltage - .15, current_voltageL, starting_voltage + .15) && ableToRegisterRev == true) {
                rotations++;
                ableToRegisterRev = false;
            }

            //has robot moved enough to register revolution
            if (ableToRegisterRev == false && !isBetween(starting_voltage - .2, current_voltageL, starting_voltage + .2)) {
                ableToRegisterRev = true;
            }

            packet.put("Voltage", current_voltageL);
            packet.put("Can add rotations", ableToRegisterRev);
            packet.put("Rotations", rotations);
            packet.put("Full Rotations", full_rotations);
            dashboard.sendTelemetryPacket(packet);
            opMode.idle();
        }

        //3.2.
        while (opMode.opModeIsActive() && !done && period.seconds() < runtime) {
            mecanum(0, low_speed_motor * dir, 0, false);

            current_voltageL = Math.floor(odometryL.getVoltage() * 1000) / 1000;

            if (isBetween(ending_voltage - .05, current_voltageL, ending_voltage + .05)) {
                done = true;
            }
            packet.put("Degrees", partial_rotations);
            packet.put("Voltage", current_voltageL);
            packet.put("Ending Voltage", ending_voltage);
            dashboard.sendTelemetryPacket(packet);
            opMode.idle();
        }

        baseBrake();

        brake();
        sleep(250);
    }
}
