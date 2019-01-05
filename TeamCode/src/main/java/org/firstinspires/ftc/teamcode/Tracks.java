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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.abs;

@TeleOp(name="Driver", group="Driver") //This is an OpMode named "Driver"
//@Disabled //This is Enabled

public class  Tracks extends LinearOpMode {
    //Define Misc. Variables
    private ElapsedTime runtime = new ElapsedTime();
    float basketPos = 180;

    //Create Motor/Servo Variables
    DcMotor RightMotor;
    DcMotor LeftMotor;
    DcMotor Arm;
    Servo Basket;
    DcMotor Collector1;
    DcMotor Collector2;
    DcMotor lifter;

    @Override
    public void runOpMode() throws InterruptedException { //When The OpMode Is Initialized
        //Tell User To Start The OpMode
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press Start to", "Start Driving");
        telemetry.update();

        //Define Motor/Servo Variables
        RightMotor = hardwareMap.dcMotor.get("motor_right");
        LeftMotor = hardwareMap.dcMotor.get("motor_left");
        Arm = hardwareMap.dcMotor.get("arm");
        Basket = hardwareMap.servo.get("basket");
        Collector1 = hardwareMap.dcMotor.get("collector1");
        Collector2 = hardwareMap.dcMotor.get("collector2");
        lifter = hardwareMap.dcMotor.get("lifter");
        //The Right Motor Must Be Reversed To Function Correctly
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Wait For The OpMode To Begin
        waitForStart();

        //Start The Runtime Timer
        runtime.reset();

        while (opModeIsActive()) { //Run The Code Until The OpMode is Stopped
            //Tell User How To Stop The OpMode
            telemetry.addData("Status", "Running");
            telemetry.addData("Press Stop to", "Stop Driving");

            //Drive On-Command
            if(gamepad1.right_stick_x==0) //When Not Told To Turn From The Right Stick
            {
                //Set Power To The Analog Value Of The Left Stick
                LeftMotor.setPower(gamepad1.left_stick_y);
                RightMotor.setPower(gamepad1.left_stick_y);
            }
            else if(gamepad1.right_stick_x>0)
            {
                //Set Power To The Analog Value
                LeftMotor.setPower(-gamepad1.right_stick_x);
                RightMotor.setPower(gamepad1.right_stick_x);
            }
            else if(gamepad1.right_stick_x<0)
            {
                LeftMotor.setPower(-gamepad1.right_stick_x);
                RightMotor.setPower(gamepad1.right_stick_x);
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
            if(gamepad2.right_stick_y!=abs(gamepad2.right_stick_y))
            {
                Collector1.setPower(gamepad2.right_stick_y * 0.75);
            }
            else
            {
                Collector1.setPower(gamepad2.right_stick_y * 0.25);
            }
            Collector2.setPower(gamepad2.left_stick_y/2);
            Basket.setPosition(basketPos/180);
            lifter.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            Arm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            telemetry.update();
        }
    }
}