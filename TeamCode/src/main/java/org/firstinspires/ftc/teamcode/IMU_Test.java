/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive TeleOp for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="IMU TEST!!", group="Linear Opmode")
@Disabled
public class IMU_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    DcMotor RightMotor;
    DcMotor LeftMotor;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' mu st correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        RightMotor = hardwareMap.dcMotor.get("motor_right");
        LeftMotor = hardwareMap.dcMotor.get("motor_left");
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation();
            Turn(-90,RightMotor,LeftMotor,imu);
            telemetry.addData("IMU Angle 1", angles.firstAngle);
            telemetry.update();
            break;
        }
    }
    //this code was added on 12/2/18
    public void Turn(float degrees, DcMotor right_Motor, DcMotor left_Motor, BNO055IMU imu)
    {
        //Declares variables
        Orientation angles = imu.getAngularOrientation();
        double delta = degrees-angles.firstAngle;
        double minSpeed= 0.55;
        //Repeats until the robot reaches position
        while(!((angles.firstAngle+delta)-1<0&&(angles.firstAngle+delta)+1>0))
        {
            //Sets he motor powers
            if(-(angles.firstAngle+delta)/100>0)left_Motor.setPower((-(angles.firstAngle+delta)/100)+minSpeed);
            else left_Motor.setPower((-(angles.firstAngle+delta)/100)-minSpeed);
            if((angles.firstAngle+delta)/100>0)right_Motor.setPower(((angles.firstAngle+delta)/100)+minSpeed);
            else right_Motor.setPower(((angles.firstAngle+delta)/100)-minSpeed);
            //Gets the currant angle
            angles = imu.getAngularOrientation();
            //Adds telemetry
            telemetry.addData("IMU Angle 1", angles.firstAngle);
            telemetry.addData("IMU with delta",angles.firstAngle+delta);
            telemetry.update();
            sleep(100);
        }
        //Stops the motors
        left_Motor.setPower(0);
        right_Motor.setPower(0);
        //DEBUG
        sleep(1000);
        angles = imu.getAngularOrientation();
        telemetry.addData("IMU Angle 1", angles.firstAngle);
        sleep(5000);
    }
    public void Forward(double distance,double power, DcMotor right_Motor, DcMotor left_Motor, BNO055IMU imu)
    {
        Orientation angles = imu.getAngularOrientation();
        float reset = -angles.firstAngle;
        float turn = angles.firstAngle-reset; //= degreesToTurn - angles.firstAngle;
        float  degrees= angles.firstAngle;
        double distancea = distance+right_Motor.getCurrentPosition();
        while(distancea*900>right_Motor.getCurrentPosition())
        {
            // DIVISION BY ZERO KINDA BREAKS THINGS
            turn         = angles.firstAngle-reset;
            angles = imu.getAngularOrientation();

            right_Motor.setPower(-(degrees-turn / 360 + ((degrees-turn / abs(degrees-turn)+0.0001) * 0.3))+power);
            left_Motor.setPower(  degrees-turn / 360 + ((degrees-turn / (abs(degrees-turn)+0.0001) * 0.3)+power));

            telemetry.addData("IMU Angle 1",      angles.firstAngle);
            telemetry.addData("IMU Angle 2",      angles.secondAngle);
            telemetry.addData("IMU Angle 3",      angles.thirdAngle);
            telemetry.addData("Degrees Variable", degrees);
            telemetry.addData("sEth is AnNoyed:: ",turn);
            telemetry.addData("sEth is stIll AnNoyed:: ",degrees-turn / 360 + ((degrees-turn / (abs(degrees-turn)+0.0001) * 0.3)+power));
            telemetry.update();
        }
        left_Motor.setPower(0);
        right_Motor.setPower(0);

    }
    private double getAngle(BNO055IMU imu)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation();

        return 0;
    }
}
