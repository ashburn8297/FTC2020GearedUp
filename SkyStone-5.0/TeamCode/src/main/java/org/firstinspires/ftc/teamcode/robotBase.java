package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static android.os.SystemClock.sleep;


public class robotBase {

    /*Public hardware members*/
    public DcMotor FLD = null; //Front Left Drive Motor, "FLD"
    public DcMotor FRD = null; //Front Right Drive Motor, "FRD"
    public DcMotor RLD = null; //Rear Left Drive Motor, "RLD"
    public DcMotor RRD = null; //Rear Right Drive Motor, "RRD"

    IntegratingGyroscope gyroNV;                //For polymorphism
    NavxMicroNavigationSensor navxMicro;        //To initialize gyroscope, "navx" on phones

    IntegratingGyroscope gyroMR;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;    //"gyro" on phones

    public static final int REV_Planetary_Ticks_Per_Rev = 1220; //How many ticks to expect per one turn of the 20:1 planetary motors.
    public static final double wheel_diameter = 4.0; //Diameter of wheel, likely dead wheel
    public static final double drive_reduction = 1.0; //This is < 1.0 if Geared UP
    public static final double ticks_per_inch = (REV_Planetary_Ticks_Per_Rev * drive_reduction) / (wheel_diameter * Math.PI);

    public static final double HEADING_THRESHOLD = 1;
    public double prev_heading = 0;

    public static final String VUFORIA_KEY = "AbEDH9P/////AAABmcFPgUDLz0tMh55QD8t9w6Bqxt3h/G+JEMdItgpjoR+S1FFRIeF/w2z5K7r/nUzRZKleksLHPglkfMKX0NltxxpVUpXqj+w6sGvedaNq449JZbEQxaYe4SU+3NNi0LBN879h9LZW9RxJFOMt7HfgssnBdg+3IsiwVKKYnovU+99oz3gJkcOtYhUS9ku3s0Wz2n6pOu3znT3bICiR0/480N63FS7d6Mk6sqN7mNyxVcRf8D5mqIMKVNGAjni9nSYensl8GAJWS1vYfZ5aQhXKs9BPM6mST5qf58Tg4xWoHltcyPp0x33tgQHBbcel0M9pYe/7ub1pmzvxeBqVgcztmzC7uHnosDO3/2MAMah8qijd";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    /*Local opMode members*/
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    //Empty constructor for object creation.
    public robotBase() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry t) {
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

        //Configure the NavXMicro for use
        navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx"); //Used for auto
        gyroNV = navxMicro;
        t.addData("gyroNV", "ONLINE");

        //Configure MR Gyro for use
        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro"); //Used for teleOp
        gyroMR = modernRoboticsI2cGyro;
        t.addData("gyroMR", "ONLINE");
        t.update();
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
    public void mecanum(double Strafe, double Forward, double Turn) {
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
        FLD.setPower(powerScale(v1));
        FRD.setPower(powerScale(v2));
        RLD.setPower(powerScale(v3));
        RRD.setPower(powerScale(v4));
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

    /**
     * A method to drive to a specific position via a desired (X,Y) pair given in inches.
     *
     * @param xInches     desired X axis translation (left right)
     * @param yInches     desired Z axis translation (front back)
     * @param timeout     maximum amount of time for the action to occur
     * @param speedBegin  how quickly to translate
     * @param endingSpeed what speed should the robot be moving at the end of the movement
     * @param opMode      the current state of the opMode
     * @param t           the current opMode's telemetry object, not opMode param
     * @TODO Design this sysetem using gyro and dead wheels
     */
    public void translate(double xInches, double yInches, double timeout, double speedBegin, double endingSpeed, LinearOpMode opMode, Telemetry t) {
        //Create a local timer
        resetEncoders();
        runUsingEncoders();

        int xTarget = (int) (xInches * 1000);
        int xTolerance = 20;

        int yTarget = (int) (yInches * 1000); //Where 1000 is ticks per inch of dead wheel in each axis
        int yTolerance = 20;

        period.reset(); //Start the clock

        //Calculate motor magnitudes , and vary between 0 & +/- 1.
        //Then apply these values over some distance based on odometry reading.

        double theta = Math.atan2(yInches, xInches) - Math.PI / 4;

        //Front Left and Right Rear
        double v1 = Math.cos(theta);
        //Front Right and Rear Left
        double v2 = Math.sin(theta);

        //How much to scale power to each pair of wheels
        double mag = 0;
   
      /*while(encoders are out of tolerance){
          calculate mag value based on (distance to target)
          apply to wheels
          report progress and values via t
      }*/


        //If requested, bring the drivebase to a full stop
        if (endingSpeed < .01) {
            brake();
            resetEncoders();
        }
    }


    /**
     * @param beginPos   the robot's beginning (starting) value
     * @param curPos     the encoder's current value
     * @param endPos     the encoder's desired ending value
     * @param beginSpeed Desired beginning speed
     * @param endSpeed   Desired ending speed
     * @return modified value for encoder ramp
     * @TODO design algorithm
     */
    public static double powerRamp(double beginPos, double curPos, double endPos, double beginSpeed, double endSpeed) {

        double pct = (curPos / (endPos - beginPos)); //0 to 1 of how far along the path in that axis the robot is
        double speed = 0.0;

        //Speed manipulation algorithm

        return speed;
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
     * @TODO Mesh with NavX
     */
    public void turn(double targetAngle, double coeff, double timeout, boolean fullStop, LinearOpMode opMode, Telemetry t) {
        //Create a local timer
        period.reset();

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
        modernRoboticsI2cGyro.resetZAxisIntegrator();

        //While active and within timeout
        while (opMode.opModeIsActive() && (period.seconds() < timeout) && (found == false)) {

            cycleHeading = gyroMR.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double c = Math.abs(cycleHeading);
            double d = Math.abs(deltaHeading);

            if (deltaHeading < cycleHeading) {
                neg = -1;
            } else {
                neg = 1;
            }

            //algorithm here  @ https://www.desmos.com/calculator/zdrnboenxp

            //Will need revision with navX gyro
            double pct = 1 + ((c - d) / d);
            if(d > 30)
                if(pct <= .95)
                    error = Math.tanh(-2 * (pct - .1)) + 1.1;
                else
                    error = (-4*pct) + 4.1;
            else
                error = Math.tanh(-2 * (pct + .2)) + 1.05;
            //IS ALWAYS OVER/UNDER THE DESIRED ANGLE, COULD BE CODE PROBLEM OR COULD BE GYROSCOPE PROBLEM, IDK
            
            //Modify speed with variable 'speed'
            steer = error * coeff * neg;

            mecanum(0.0, 0.0, steer);

            //Terminating condition
            if (Math.abs(cycleHeading - deltaHeading) < HEADING_THRESHOLD) {
                found = true;
            }

            t.addData("Current Angle", cycleHeading);
            t.addData("Target Angle", deltaHeading);
            t.addData("Time", "%.1f", period.seconds());
            t.update();
            opMode.idle();
        }
        sleep(250);
        prev_heading = targetAngle;
        if (fullStop == true) {
            brake();
        }
    }

    public void gyroData(double timeout, LinearOpMode opMode, Telemetry t) {
        period.reset(); //Start the clock
        while (period.seconds() < timeout && opMode.opModeIsActive()) {

            int heading = modernRoboticsI2cGyro.getHeading();
            int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
            float zAngle = gyroMR.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            t.addData("Heading", heading);
            t.addData("Integrated Z", integratedZ);
            t.addData("Z Angle", zAngle);
            t.addData("Time", "%.1f", period.seconds());
            t.update();
            opMode.idle();
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
}
