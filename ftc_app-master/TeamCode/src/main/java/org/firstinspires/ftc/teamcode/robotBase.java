package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public class robotBase {

    /*Public hardware members*/
    public DcMotor FLD                  = null; //Front Left Drive Motor, "FLD"
    public DcMotor FRD                  = null; //Front Right Drive Motor, "FRD"
    public DcMotor RLD                  = null; //Rear Left Drive Motor, "RLD"
    public DcMotor RRD                  = null; //Rear Right Drive Motor, "RRD"

    IntegratingGyroscope gyro;                  //For polymorphism
    NavxMicroNavigationSensor navxMicro;        //To initialize gyroscope

    public static final int REV_Planetary_Ticks_Per_Rev = 1220; //How many ticks to expect per one turn of the 20:1 planetary motors.
    public static final double wheel_diameter           = 4.0; //Diameter of wheel
    public static final double drive_reduction          = 1.0; //This is < 1.0 if Geared UP
    public static final double ticks_per_inch           = (REV_Planetary_Ticks_Per_Rev * drive_reduction)/(wheel_diameter * Math.PI);

    public static final double HEADING_THRESHOLD        = .75 ;

    public static final String VUFORIA_KEY = "AbEDH9P/////AAABmcFPgUDLz0tMh55QD8t9w6Bqxt3h/G+JEMdItgpjoR+S1FFRIeF/w2z5K7r/nUzRZKleksLHPglkfMKX0NltxxpVUpXqj+w6sGvedaNq449JZbEQxaYe4SU+3NNi0LBN879h9LZW9RxJFOMt7HfgssnBdg+3IsiwVKKYnovU+99oz3gJkcOtYhUS9ku3s0Wz2n6pOu3znT3bICiR0/480N63FS7d6Mk6sqN7mNyxVcRf8D5mqIMKVNGAjni9nSYensl8GAJWS1vYfZ5aQhXKs9BPM6mST5qf58Tg4xWoHltcyPp0x33tgQHBbcel0M9pYe/7ub1pmzvxeBqVgcztmzC7uHnosDO3/2MAMah8qijd";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

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

        //Configure the NavXMicro for use
        //Be sure to Factory reset occasionally
        //Link -> https://pdocs.kauailabs.com/navx-micro/wp-content/uploads/2019/02/navx-micro_robotics_navigation_sensor_user_guide.pdf Page 35
        navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");

        //https://pdocs.kauailabs.com/navx-micro/guidance/gyroaccelerometer-calibration/
        gyro = navxMicro;

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



    public void resetEncoders(){
        FLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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



     /**Control a mecanum drive base with three double inputs
     *
     * @param Strafe is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn is the second double X value which represents how the base should turn
     * */
    public void mecanum(double Strafe, double Forward, double Turn){
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



    /**Strafe a mecanum drive where no turn is sent, robot attempts to maintain heading
     *
     * @param Strafe is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Heading is the heading the robot should maintain
     * */
    public void mecanumGyroLock(double Strafe, double Forward, double Heading) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double current = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //This may need to be reversed
        double error = (Heading-current)/180;

        //if error is positive, spin negative
        //if error is negative, spin positive

        //double vX represents the velocities sent to each motor
        final double v1 = r * Math.cos(robotAngle) + error;
        final double v2 = r * Math.sin(robotAngle) - error;
        final double v3 = r * Math.sin(robotAngle) + error;
        final double v4 = r * Math.cos(robotAngle) - error;

        //Ramp these values with powerScale's values.
        FLD.setPower(powerScale(v1));
        FRD.setPower(powerScale(v2));
        RLD.setPower(powerScale(v3));
        RRD.setPower(powerScale(v4));
    }



    /**Scale input to a modified sigmoid curve
     *
     * @param power is a double input from the mecanum method
     *
     * @return modified power value per the sigmoid curve
     * graph here -> https://www.desmos.com/calculator/c6uav7pudi
     */
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



    /**A method to drive to a specific position via a desired (X,Z) pair given in inches.
     *
     * @param xInches desired X axis translation (left right)
     * @param zInches desired Z axis translation (front back)
     * @param timeout maximum amount of time for the action to occur
     * @param speed how quickly to translate
     * @param opMode the current state of the opMode
     */
    public void translate(double xInches, double zInches, double timeout, double speed, LinearOpMode opMode){
        //Create a local timer
        ElapsedTime period  = new ElapsedTime();

        runToPosition();
        resetEncoders();

        int FLTarget = (int) ((zInches * ticks_per_inch) - (xInches * ticks_per_inch));
        int FRTarget = (int) ((zInches * ticks_per_inch) + (xInches * ticks_per_inch));
        int RLTarget = (int) ((zInches * ticks_per_inch) + (xInches * ticks_per_inch));
        int RRTarget = (int) ((zInches * ticks_per_inch) - (xInches * ticks_per_inch));

        FLD.setTargetPosition(FLTarget);
        FRD.setTargetPosition(FRTarget);
        RLD.setTargetPosition(RLTarget);
        RRD.setTargetPosition(RRTarget);

        period.reset();
        while((period.seconds() < timeout) && (FLD.isBusy() || FRD.isBusy() || RLD.isBusy() || RRD.isBusy()) && opMode.opModeIsActive()){

            double curPos = (Math.abs(FLD.getCurrentPosition())+Math.abs(FRD.getCurrentPosition())+Math.abs(RLD.getCurrentPosition())+Math.abs(RRD.getCurrentPosition()))/4; //The average robot's current position
            double endPos = (Math.abs(FLTarget)+Math.abs(FRTarget)+Math.abs(RLTarget)+Math.abs(RRTarget))/4;//The average robot's ending position

            /*
             * This method of position tracking works because encoders are reset.
             * Mecanum slippage may undershoot these distances, check vectoring.
             */

            //Using average positions, compute system's velocity
            double vel = powerRamp(Math.abs(speed), curPos, endPos);
            FLD.setPower(vel);
            FRD.setPower(vel);
            RLD.setPower(vel);
            RRD.setPower(vel);
        }

        //Stop when done
        brake();

        //Revert to previous running state
        runUsingEncoders();
    }



    /**
     *
     * @param speed maximum desired speed (between 0 & 1)
     * @param curPos the encoder's current value
     * @param endPos the encoder's desired ending value
     * @return modified value for encoder ramp
     *
     * graph here -> https://www.desmos.com/calculator/bprnntmxxz
     */
    public static double powerRamp(double speed, double curPos, double endPos){
        double pos = curPos/endPos; //provides double of percentage to end
        if(pos < .5){
            return speed * ((1.5 * pos) + .25);
        }
        else{
            return speed * ((-1.5 * pos) + 1.75);
        }
    }



    /**Turn towards a given heading
     *
     * @param targetAngle desire heading post-turn
     * @param speed how quickly to turn about the point
     * @param timeout maximum amount of time for the action to occur
     * @param opMode the current state of the opMode
     */
    public void turn(double targetAngle, double speed, double timeout, LinearOpMode opMode) {
        //Create a local timer
        ElapsedTime period = new ElapsedTime();
        double  error;
        double  steer;

        //Coefficient of turn
        double  PCoeff = .015;

        //While active and within timeout
        while(opMode.opModeIsActive() && (period.seconds() < timeout)){
            //Pull the robot's current heading
            double CurAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            //Find the error
            error = targetAngle - CurAngle;

            //If within threshold, stop.
            if (Math.abs(error) <= HEADING_THRESHOLD) {
                brake();
            }

            //Else use coeff to steer and set power.
            else {
                steer = Range.clip(error * PCoeff, -1, 1);
                speed  = speed * steer;
                mecanum(0.0, 0.0, speed);
            }
            opMode.idle();
        }
        //Stop when done.
        brake();
    }

    //Placeholders
    //initTfod needs new assets

    public void startupIR(){
        initTfod();
        initVuforia();
    }
    public void startTFOD(){
        tfod.activate();
    }
    public void stopTFOD(){
        tfod.deactivate();
    }
    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.useExtendedTracking = false;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }
    private void initTfod () {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = .4;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        //These models need to be replaced
        //https://codelabs.developers.google.com/codelabs/tensorflow-for-poets-2-tflite/#1
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}