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

import android.sax.TextElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Random;

import static java.lang.Math.abs;

@TeleOp(name="Drive", group="Drive")
//@Disabled
public class  Tracks extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor RightMotor;
    DcMotor LeftMotor;
    DcMotor Arm;
    Servo Basket;
    DcMotor Collector1;
    DcMotor Collector2;
    BNO055IMU imu;
    DistanceSensor distanceSensor;
    Orientation angles;
    float basketPos = 180;
    Random rand = new Random();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        RightMotor = hardwareMap.dcMotor.get("motor_right");
        LeftMotor = hardwareMap.dcMotor.get("motor_left");
        Arm = hardwareMap.dcMotor.get("arm");
        Basket = hardwareMap.servo.get("basket");
        Collector1 = hardwareMap.dcMotor.get("collector1");
        Collector2 = hardwareMap.dcMotor.get("collector2");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Most robots need the motor on one side to be reversed to drive forward
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            if(gamepad1.right_stick_x==0)
            {
                LeftMotor.setPower(gamepad1.left_stick_y);
                RightMotor.setPower(gamepad1.left_stick_y);
            }
            else if(gamepad1.right_stick_x>0)
            {
                LeftMotor.setPower(-gamepad1.right_stick_x);
                RightMotor.setPower(gamepad1.right_stick_x);
            }
            else if(gamepad1.right_stick_x<0)
            {
                LeftMotor.setPower(-gamepad1.right_stick_x);
                RightMotor.setPower(gamepad1.right_stick_x);
            }
            if(gamepad2.left_trigger>0)
            {
                Arm.setPower(gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger>0)
            {
                Arm.setPower(-gamepad2.right_trigger);
            }
            else
            {
                Arm.setPower(0);
            }
            if(gamepad1.y)
            {
                basketPos=50;
            }
            if(gamepad1.x)
            {
                basketPos=180;
            }
            if(gamepad2.b)
            {
                basketPos=150;
            }
            if(gamepad1.b)
            {
                basketPos=50;
                Basket.setPosition(basketPos/180);
                Thread.sleep(750);
                basketPos=180;
            }
            if(gamepad2.right_stick_y<0)
            {
                Collector1.setPower(gamepad2.right_stick_y * 0.75);
            }
            else
            {
                Collector1.setPower(gamepad2.right_stick_y * 0.25);
            }
            Collector2.setPower(gamepad2.left_stick_y/2);
            Basket.setPosition(basketPos/180);
        }
    }
}