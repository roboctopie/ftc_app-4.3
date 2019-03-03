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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp(name="Lifter Test", group="Test")
//@Disabled
public class Lifter_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor lifter;
    DcMotor CollectorLift;
    DcMotor Collector;
    int start = 0;
    double prevStep;
    double stepsPerCycle = 0.6;
    double powerAdjust = 0;
    int cycleLength = 50;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lifter = hardwareMap.dcMotor.get("Tentacle_M");
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CollectorLift = hardwareMap.dcMotor.get("collector1");
        Collector = hardwareMap.dcMotor.get("collector2");
        waitForStart();
        runtime.reset();

        //while (opModeIsActive()) {
        //lifter.setPower(gamepad1.left_stick_y);
        start = lifter.getCurrentPosition();
        prevStep = start;

        while(lifter.getCurrentPosition()<start+76&&opModeIsActive())
        {
            powerAdjust = -sigmoid(stepsPerCycle-(lifter.getCurrentPosition()-prevStep))*0.36;
            lifter.setPower(lifter.getPower()+powerAdjust+0.15);
            sleep(cycleLength);
            telemetry.addData("",powerAdjust);
            telemetry.update();
        }
        lifter.setPower(0);
        sleep(250);
        CollectorLift.setPower(-0.5);
        sleep(420);
        CollectorLift.setPower(0);
        sleep(100);
        Collector.setPower(1);
        sleep(800);
        Collector.setPower(0);
        sleep(100);
        CollectorLift.setPower(0.5);
        sleep(1234);
        CollectorLift.setPower(0);
        sleep(250);
        /*
        while(lifter.getCurrentPosition()>start+80&&opModeIsActive())
        {
            powerAdjust = -sigmoid(-stepsPerCycle-(lifter.getCurrentPosition()-prevStep))*0.36;
            lifter.setPower(lifter.getPower()+powerAdjust-0.15);
            sleep(cycleLength);
            telemetry.addData("",powerAdjust);
            telemetry.update();
        }
        lifter.setPower(0);
        */
        sleep(10000);
    }
    public static double sigmoid(double input)
    {
        return (1/(1+Math.pow(1.5,-(input))))*-0.5;
    }
}
