package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static java.lang.Thread.sleep;

public class robotBase {

    /*Public hardware members*/
    public DcMotor FLD                  = null; //Front Left Drive Motor, "FLD"
    public DcMotor FRD                  = null; //Front Right Drive Motor, "FRD"
    public DcMotor RLD                  = null; //Rear Left Drive Motor, "RLD"
    public DcMotor RRD                  = null; //Rear Right Drive Motor, "RRD"

    BNO055IMU imu;                              //REV Expansion Hub's internal IMU

    public int REVPlanetaryTicksPerRev  = 1220; //How many ticks to expect per one turn of the 20:1 planetary motors.

    /*Local opMode members*/
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    //Empty constructor for object creation.
    public robotBase(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Initialize the four drive base motors
        FLD = hwMap.get(DcMotor.class, "FLD");
        FRD = hwMap.get(DcMotor.class, "FRD");
        RLD = hwMap.get(DcMotor.class, "RLD");
        RRD = hwMap.get(DcMotor.class, "RRD");

        //Set the drive base motors so that +1 drives forward
        /*It's possible these values are reversed*/
        FLD.setDirection(DcMotor.Direction.FORWARD);
        FRD.setDirection(DcMotor.Direction.REVERSE);
        RLD.setDirection(DcMotor.Direction.FORWARD);
        RRD.setDirection(DcMotor.Direction.REVERSE);

        //Stop all motors when robot is initialized
        brake();

        //Set all motors so that they begin without encoders enabled
        runWithoutEncoders();

        //Set all motors so when zero power is issues, motors to not actively resist (brake)
        baseFloat();

        //Configure the Bosch IMU for use
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Run the robot, ignoring encoder values
    public void runWithoutEncoders(){
        FLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Run the robot using encoder values as velocity check
    public void runUsingEncoders(){
        FLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Run the robot using ecoders to run to a specified position (encoder ticks);
    public void runToPosition(){
        FLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Set all motors to float when zero power command is issued
    public void baseFloat(){
        FLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Set all motors to actively resist when zero power command is issued
    public void baseBrake(){
        FLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Set all motors to zero power
    public void brake(){
        FLD.setPower(0);
        FRD.setPower(0);
        RLD.setPower(0);
        RRD.setPower(0);
    }

    /*Wheels must be instaled in X pattern (based on rollers) */
    /*Control the mecanum drive base with a controller's x input (double strafe)
    and a controller's y input (double forward) to translate,
    and a second x input (double turn) to rotate*/
    /*Positive inputs represent a right translation, forward movement, and clockwise turn */
    public void mecanumController(double Strafe, double Forward, double Turn){
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

    //Scale input to a modified sigmoid curve
    public static double powerScale(double power){
        //If power is negative, log this and save for later
        //If positive, keep neg to 1, so that num is always positive
        //If negative, change to -1, so that ending number is negative
        double neg = 1;
        if(power < 0)
            neg = -1;

        //Set power to always be positive
        power = Math.abs(power);

        //modify to sigmoid
        power = 1.05/(1 + Math.pow(Math.E, -10*(power-.5)));
        //graph here -> https://www.desmos.com/calculator/c6uav7pudi

        //Make sure value falls between -1 and 1
        return Range.clip(power * neg, -1, 1);
    }

    //A method to drive to a specific position via a desired (X,Y) pair given in inches.
    //https://github.com/trc492/FtcSamples/blob/master/Ftc3543Lib/src/main/java/trclib/TrcPidDrive.java

    //Get the IMU's current x value
    public double getIMUPositionX(){
        return imu.getPosition().unit.toInches(imu.getPosition().x);
    }

    //Get the IMU's current z value
    public double getIMUPositionZ(){
        return imu.getPosition().unit.toInches(imu.getPosition().z);
    }

    public Orientation getIMUAngles(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
