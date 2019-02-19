/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 /*
 * This i s roboctopi's V1 software for autonomous.
 * It was used at our first competition on 12/9/18 during the morning session at Francis Parker High School
 * This code runs these tasks in this order:
 * (1.) Detects the minerals in the sampling position
 * (2.) If the gold is in the center position the robot:
 *  (2b.) Drives forward and pushes the gold mineral out of the way and into crater
 * (3.) If the robot detects that the gold is in the right or left position:
 *  (3a.) Drives forward a small amount
 *  (3b.) Turns 30° right or left depending on the gold position
 *  (3c.) Drives forward to move the gold mineral out of the way
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Block Detector Crater", group = "Block Detector")
//@Disabled
public class BlockDetectCrater extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private ElapsedTime runtime = new ElapsedTime();
    Orientation      lastAngles = new Orientation();
    double globalAngle,   power = .30, correction;
    DcMotor RightMotor;
    DcMotor LeftMotor;
    DcMotor Arm;
    Servo Basket;
    Servo Straw;
    DcMotor CollectorLift;
    DcMotor Collector;
    BNO055IMU imu;
    Orientation angles;

    private DistanceSensor distanceSensor;
    float basketPos = 185;
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AW/aXP//////AAABmUL6p+56MU6rnqNxEkLhIJgblkVek4ygJheFrXngFLjvVakXKX/d9N3+Wtybm3PrmjrNzp607yhCTqDte6AjO3rMnKs9ZsEi2j63SySN51RRwuwxdqHkh1vAP+2pIMBtlGISJEMCnIX5PQ0dlbt7GUS16ca0vqkvqDwyN9/OBVbMlYrawCw8ttpOXWjxl4pDnqodRByM+LOMKVANu/jYjkSzDwuX79yw8kUqEDEOytePg8C+3is7mfwI93zTwv3s72Wn7YTF9G9Vkcf9h5y5qFEdP3liEm+V/sibPYsL+TWkZPz4N6XImJtCvcxGKe+4FW9gn40Q0YtKA6kuk8EX+5tjpQVzfmzknvwjE2abewZy";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        RightMotor = hardwareMap.dcMotor.get("motor_right");
        LeftMotor = hardwareMap.dcMotor.get("motor_left");
        Arm = hardwareMap.dcMotor.get("arm");
        Basket = hardwareMap.servo.get("basket");
        CollectorLift = hardwareMap.dcMotor.get("collector1");
        Collector = hardwareMap.dcMotor.get("collector2");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        Straw = hardwareMap.servo.get("Lego");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        // Most robots need the motor on one side to be reversed to drive forward
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parametersVu.vuforiaLicenseKey = VUFORIA_KEY ;
        parametersVu.cameraDirection   = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parametersVu);
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        /** Wait for the game to begin */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press Start to", "Start Detecting");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            //(1.) Detects the minerals in the sampling position
                            //We added recognition.getTop()>500 to the if statement to ignore the minerals in the crater
                            //12-8-18 Fixed detection of minerals in crater.
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)&&recognition.getTop()>500) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1&&recognition.getTop()>500) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else if(recognition.getTop()>500) {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                             //We had to modify the code that decided which position the gold was in because our robot can only see two minerals at the sme time
                            if ((goldMineralX != -1 && silverMineral1X != -1) ||(silverMineral2X != -1&&goldMineralX != -1)||(silverMineral2X != -1&&silverMineral1X != -1)) {
                                telemetry.addData("a",goldMineralX);
                                telemetry.addData("a",silverMineral1X);
                                telemetry.addData("a",silverMineral2X);
                                //(3.) If the robot detects that the gold is in the right or left position:
                                if ((goldMineralX < silverMineral1X || goldMineralX < silverMineral2X) && goldMineralX != -1) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.update();
                                    tfod.deactivate();
                                    //(3a.) Drives forward a small amount
                                    forward(2,0.5);
                                    //(3b.) Turns 30° right or left depending on the gold position
                                    rotate(30,0.5);
                                    //(3c.) Drives forward to move the gold mineral out of the way
                                    forward(5,0.5);
                                    forward(-5,0.5);
                                    rotate(26,0.65);
                                    forward(6.3,0.5);
                                    targetsRoverRuckus.activate();

                                    rotate(getDegToTurn(allTrackables),0.5);

                                    while (distanceSensor.getDistance(DistanceUnit.INCH) > 10.5)
                                    {
                                        RightMotor.setPower(0.5);
                                        LeftMotor.setPower(0.5);
                                    }
                                    rotate(75,0.65);
                                    forward(8,0.5);

                                    FrunkDown();
                                    Barf();
                                    FrunkUp();
                                    forward(-5,0.5);
                                    rotate(7,.65);
                                    forward(-5,0.5);
                                    Straw.setPosition(0.5);
                                    break;
                                //(2.) If the gold is in the center position the robot:
                                } else if (goldMineralX > silverMineral1X || goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.update();
                                    tfod.deactivate();
                                    //2b.) Drives forward and pushes the gold mineral out of the way and into crater
                                    forward(7,0.5);
                                    forward(-4.2,0.5);
                                    rotate(58,0.65);
                                    forward(6.2,0.5);
                                    targetsRoverRuckus.activate();

                                    rotate(getDegToTurn(allTrackables),0.5);

                                    while (distanceSensor.getDistance(DistanceUnit.INCH) > 10.5)
                                    {
                                        RightMotor.setPower(0.5);
                                        LeftMotor.setPower(0.5);
                                    }
                                    rotate(75,0.65);
                                    forward(8,0.5);

                                    FrunkDown();
                                    Barf();
                                    FrunkUp();
                                    forward(-5,0.5);
                                    rotate(7,.65);
                                    forward(-5,0.5);
                                    Straw.setPosition(0.5);
                                    break;
                                } else { telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.update();
                                    tfod.deactivate();
                                    forward(2,0.5);
                                    rotate(-32,0.5);
                                    forward(6,0.5);
                                    forward(-6,0.5);
                                    rotate(90,0.65);
                                    forward(6,0.5);
                                    targetsRoverRuckus.activate();

                                    rotate(getDegToTurn(allTrackables),0.5);

                                    while (distanceSensor.getDistance(DistanceUnit.INCH) > 10.5)
                                    {
                                        RightMotor.setPower(0.5);
                                        LeftMotor.setPower(0.5);
                                    }
                                    rotate(75,0.65);
                                    forward(8,0.5);

                                    FrunkDown();
                                    Barf();
                                    FrunkUp();
                                    forward(-5,0.5);
                                    rotate(7,.65);
                                    forward(-5,0.5);
                                    Straw.setPosition(0.5);
                                    break;

                                }
                            }
                        }
                        telemetry.update();
                    }
                }

            }

        }

        }



    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(double degrees, double power)
    {
        double  leftPower, rightPower;
        sleep(150);
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower =  power;
            rightPower =-power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        LeftMotor.setPower(leftPower);
        RightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees){}

        // turn the motors off.
        RightMotor.setPower(0);
        LeftMotor.setPower(0);
        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private  void forward(double distance,double power)
    {
        double pos = RightMotor.getCurrentPosition();
        if(distance>0)
        {
            while (RightMotor.getCurrentPosition() < pos + (distance * 300))
            {
                RightMotor.setPower(power);
                LeftMotor.setPower(power);
            }
        }
        else
        {
            while (RightMotor.getCurrentPosition() > pos + (distance * 300))
            {
                RightMotor.setPower(-power);
                LeftMotor.setPower(-power);
            }
        }


        RightMotor.setPower(0);
        LeftMotor.setPower(0);
    }
    public void FrunkDown() {
        CollectorLift.setPower(-1);
        sleep(650);
        CollectorLift.setPower(0);
    }
    public void FrunkUp() {

        CollectorLift.setPower(1);
        sleep(800);
        CollectorLift.setPower(0);
    }
    public double getDegToTurn(List<VuforiaTrackable>allTrackables)
    {

        double degreesToTurn = 0;
        for(int x = 0;x<200;x++) {
            for (VuforiaTrackable trackable : allTrackables) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), -translation.get(2)));
                }

            }
            sleep(7);
            telemetry.addData("test",degreesToTurn);
            telemetry.update();
        }

        return degreesToTurn;
    }
    public void Barf()
    {
        //(2c.) Spits out our team marker into the depot
        Collector.setPower(1);
        sleep(800);
        Collector.setPower(0);
    }
}
