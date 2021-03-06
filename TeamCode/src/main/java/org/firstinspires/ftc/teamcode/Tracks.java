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
    float tentacleSPos = -5;

    //Create Motor/Servo Variables
    DcMotor RightMotor;
    DcMotor LeftMotor;
    DcMotor Arm;
    Servo Basket;
    DcMotor Collector1;
    DcMotor Collector2;
    DcMotor lifter;
    DcMotor TentacleM;
    Servo LegoWheel;
    double turnMultipliyer;
    //Servo TentacleS;

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
        TentacleM = hardwareMap.dcMotor.get("Tentacle_M");
        //TentacleS = hardwareMap.servo.get("Tentacle_S");
        //The Right Motor Must Be Reversed To Function Correctly
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        TentacleM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LegoWheel = hardwareMap.servo.get("Lego");
        //TentacleS.setPosition(tentacleSPos/180);
        //Wait For The OpMode To Begin
        LegoWheel.setPosition(1);
        waitForStart();

        //Start The Runtime Timer
        runtime.reset();

        while (opModeIsActive()) { //Run The Code Until The OpMode is Stopped
            //Tell User How To Stop The OpMode
            telemetry.addData("Status", "Running");
            telemetry.addData("Press Stop to", "Stop Driving");

            //Drive On-Command
            /*if(gamepad1.right_stick_x==0) //When Not Told To Turn From The Right Stick
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
            */
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
                basketPos=124;
            }

            Collector1.setPower(-gamepad2.right_stick_y);
            Collector2.setPower(gamepad2.left_stick_y/2);
            Basket.setPosition(basketPos/180);
            TentacleM.setPower((gamepad1.left_trigger*0.4)-(gamepad1.right_trigger*0.75));
            Arm.setPower((gamepad2.right_trigger*0.75)-(gamepad2.left_trigger*0.75));
            /*if(abs(gamepad1.right_stick_x) < 0.1) {
                //If the drivers are not turning the robot moves faster because it can
                LeftMotor.setPower(gamepad1.left_stick_y);
                RightMotor.setPower(gamepad1.left_stick_y);
            } else if (abs(gamepad1.left_stick_y) > 0.1) {
                //If the drivers are going forward and turning
                */
                //This does arc-turning
            //Add sign param
           /* if(Zero(abs(gamepad1.left_stick_y) - gamepad1.right_stick_x)==0) {
                LeftMotor.setPower(gamepad1.right_stick_x);
                RightMotor.setPower(-gamepad1.right_stick_x);
            } else {
                LeftMotor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) - gamepad1.right_stick_x);
                RightMotor.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) + gamepad1.right_stick_x);
            }


                } else {

            }*/
            turnMultipliyer = (Limit(abs(gamepad1.right_stick_x)+abs(gamepad1.left_stick_y))/(abs(gamepad1.right_stick_x)+abs(gamepad1.left_stick_y)));
            telemetry.addData("turnMultiplier / Crazy Bob", turnMultipliyer);
            LeftMotor.setPower((gamepad1.left_stick_y*turnMultipliyer)-(gamepad1.right_stick_x*turnMultipliyer));
            telemetry.addData("LeftMotor Speed", (gamepad1.left_stick_y*turnMultipliyer)-(gamepad1.right_stick_x*turnMultipliyer));
            RightMotor.setPower((gamepad1.left_stick_y*turnMultipliyer)+(gamepad1.right_stick_x*turnMultipliyer));
            telemetry.addData("RightMotor Speed", (gamepad1.left_stick_y*turnMultipliyer)+(gamepad1.right_stick_x*turnMultipliyer));
            if(gamepad1.left_bumper) {
                lifter.setPower(1);
            }
            else if(gamepad1.right_bumper)
            {
                lifter.setPower(-1);
            }
            else
            {
                lifter.setPower(0);
            }
            if(gamepad2.a) {
                tentacleSPos = 77;
            }
            else if(gamepad2.x) {
                tentacleSPos = 170;
            }
            //TentacleS.setPosition(tentacleSPos/180);
            telemetry.update();
        }
    }
    public static double Limit(double input)
    {
        if(input>1) return 1;
        else return input;
    }
}