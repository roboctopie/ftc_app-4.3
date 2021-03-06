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

/*
 * Copyright (c) 2018 Team Roboctopi (#14496)
 * ------------------------------------------
 * Driver Code v1
 *
 * Used on Dec 09, 2018
 * @ Francis Parker School
 * Ptolemy League Competition
 * Morning Session
 *
 * Hardware:
 *  • 5 Motors
 *  • 1 Servo
 * Controls:
 *  • Driver 1:
 *    • Left Stick Up:     Drive Forward
 *      • Analog Stick To Control Speed So Drivers Have Accurate Driving
 *    • Left Stick Down:   Drive Backward
 *      • Analog Stick To Control Speed So Drivers Have Accurate Driving
 *    • Right Stick Left:  Turn Left
 *      • Analog Stick To Control Speed So Drivers Have Accurate Turning
 *      • Easier If Turning Is Seperate From Driving For Drivers
 *    • Right Stick Right: Turn Right
 *      • Analog Stick To Control Speed So Drivers Have Accurate Turning
 *      • Easier If Turning Is Seperate From Driving For Drivers
 *    • Y Button:          Dump Basket
 *      • Button To Go To The Same Place Every Time
 *    • X Button:          Return Basket From Dump
 *      • Button To Go To The Same Place Every Time
 *    • B Button:          Dump And Return Basket
 *      • Button To Go To The Same Place Every Time
 *  • Driver 2:
 *    • Left Trigger:      Arm Down
 *      • Analog Trigger So We Have Fine Control Over The Arm
 *    • Right Trigger:     Arm Up
 *      • Analog Trigger So We Have Fine Control Over The Arm
 *    • Right Stick Up:    Collector Up
 *      • Analog Stick So It Doesn't Go Too Fast And Hit The Robot Too Hard
 *    • Right Stick Down:  Collector Down
 *      • Analog Stick So It Doesn't Go Too Fast And Hit The Playing Field Too Hard
 *    • Left Stick Up:     Collect Minerals
 *      • Analog Stick to Control How Many Minerals Are Collected
 *    • Right Stick Down:  Reject Minerals
 *      • Analog Stick to Control How Many Minerals Are Rejected
 *    • B Button:          Clear Basket for Arm Up
 *      • Button To Go To The Same Place Every Time
 * ------------------------------------------
 */

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import static java.lang.Math.abs;

@TeleOp(name="Driver Troubleshooter", group="Driver")
//@Disabled
public class  Tracks_DEVTEST extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press Start to", "Start Troubleshooting");
        telemetry.update();

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

        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Press Stop to", "Stop Troubleshooting\n");
            telemetry.addData("Motor Power Data", "");
            if(gamepad1.right_stick_x==0)
            {
                LeftMotor.setPower(gamepad1.left_stick_y);
                telemetry.addData("      Left Motor Power", gamepad1.left_stick_y);
                RightMotor.setPower(gamepad1.left_stick_y);
                telemetry.addData("      Right Motor Power", gamepad1.left_stick_y);
            }
            else if(gamepad1.right_stick_x>0)
            {
                LeftMotor.setPower(-gamepad1.right_stick_x);
                telemetry.addData("      Left Motor Power", -gamepad1.right_stick_x);
                RightMotor.setPower(gamepad1.right_stick_x);
                telemetry.addData("      Right Motor Power", gamepad1.right_stick_x);
            }
            else if(gamepad1.right_stick_x<0)
            {
                LeftMotor.setPower(-gamepad1.right_stick_x);
                telemetry.addData("      Left Motor Power", -gamepad1.right_stick_x);
                RightMotor.setPower(gamepad1.right_stick_x);
                telemetry.addData("      Right Motor Power", gamepad1.right_stick_x);
            }
            if(gamepad2.left_trigger>0)
            {
                Arm.setPower(gamepad2.left_trigger);
                telemetry.addData("      Arm Power", gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger>0)
            {
                Arm.setPower(-gamepad2.right_trigger);
                telemetry.addData("      Arm Power", -gamepad2.right_trigger);
            }
            else
            {
                Arm.setPower(0);
                telemetry.addData("      Arm Power", "0.0");
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
            if(gamepad1.b) {
                basketPos = 50;
                Basket.setPosition(basketPos / 180);
                Thread.sleep(750);
                basketPos = 180;
            }
            if(gamepad2.left_stick_y!=abs(gamepad2.left_stick_y))
            {
                Collector1.setPower(gamepad2.left_stick_y * 0.75);
                telemetry.addData("      Collector Motor Power", gamepad2.left_stick_y * 0.25);
            }
            else
            {
                Collector1.setPower(gamepad2.left_stick_y * 0.25);
                telemetry.addData("      Collector Motor Power", gamepad2.left_stick_y * 0.75);
            }
            Collector2.setPower(gamepad2.right_stick_y/2);
            telemetry.addData("      Collector Brushes Power", gamepad2.right_stick_y/2 + "\n");
            Basket.setPosition(basketPos/180);
            telemetry.addData("Motor Position Data", "");
            telemetry.addData("      Left Motor Position", LeftMotor.getCurrentPosition());
            telemetry.addData("      Right Motor Position", RightMotor.getCurrentPosition());
            telemetry.addData("      Arm Position", Arm.getCurrentPosition());
            telemetry.addData("      Collector Motor Position", Collector1.getCurrentPosition());
            telemetry.addData("      Collector Brushes Position", Collector2.getCurrentPosition());
            telemetry.addData("      Basket Position", Basket.getPosition() + "\n");
            telemetry.addData("Gamepad 1 Data", "");
            telemetry.addData("      Gamepad 1 Type", gamepad1.type());
            telemetry.addData("      Gamepad 1 B Pressed", gamepad1.b);
            telemetry.addData("      Gamepad 1 X Pressed", gamepad1.x);
            telemetry.addData("      Gamepad 1 Y Pressed", gamepad1.y);
            telemetry.addData("      Gamepad 1 LB Pressed", gamepad1.left_bumper);
            telemetry.addData("      Gamepad 1 LT Position", gamepad1.left_trigger);
            telemetry.addData("      Gamepad 1 RT Position", gamepad1.right_trigger);
            telemetry.addData("      Gamepad 1 Left Stick Y Position", gamepad1.left_stick_y);
            telemetry.addData("      Gamepad 1 Right Stick X Position", gamepad1.right_stick_x + "\n");
            telemetry.addData("Gamepad 2 Data", "");
            telemetry.addData("      Gamepad 2 Type", gamepad2.type());
            telemetry.addData("      Gamepad 2 B Pressed", gamepad2.b);
            telemetry.addData("      Gamepad 2 Left Stick Y Position", gamepad2.left_stick_y);
            telemetry.addData("      Gamepad 2 Right Stick Y Position", gamepad2.right_stick_y + "\n");
            telemetry.addData("Sensor Data", "");
            angles = imu.getAngularOrientation();
            telemetry.addData("      IMU Angle 1", angles.firstAngle);
            telemetry.addData("      IMU Angle 2", angles.secondAngle);
            telemetry.addData("      IMU Angle 3", angles.thirdAngle);
            telemetry.addData("      Distance to Obstacle (in)", distanceSensor.getDistance(DistanceUnit.INCH) + "\n");
            telemetry.addData("Miscellaneous", "");
            telemetry.addData("      License", "Read/Write/Run");
            telemetry.addData("      Run Time", runtime);
            telemetry.addData("      Number of Data Points", "32.0");
            telemetry.update();
        }
    }
}