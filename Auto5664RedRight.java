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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto5664RedRight", group="Pushbot")

public class Auto5664RedRight extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareCade robot = new HardwareCade();   // Use a Cade's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static boolean bluefound = false;
    static boolean redfound = false;

    @Override
    public void runOpMode()
    {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftfront.getCurrentPosition(),
                robot.rightback.getCurrentPosition(),
                robot.leftback.getCurrentPosition(),
                robot.rightfront.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //1) Close servo
        robot.rightdownservo.setPosition(-0.5);
        robot.leftdownservo.setPosition(0.5);
        sleep(1000);

        //2)Raise Front Claw a tiny bit
        runtime.reset();
        robot.frontclawlift.setPower(-0.5);
        while (opModeIsActive() && runtime.seconds() < 0.5)
        {
        }
        robot.frontclawlift.setPower(0);
        sleep(500);
        //3)Drop the ColorServo in a forward direction
        robot.sideservo.setPosition(0.08);
        sleep(750);
        //4)Have Color read for the coresponding color
        robot.ColorS.enableLed(true);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2)
        {
            telemetry.addData("SenseJewel", "Red:" + robot.ColorS.red());
            telemetry.addData("SenseJewel", "Blue:" + robot.ColorS.blue());
            telemetry.update();

            idle();
        }
        if (robot.ColorS.red() >= 1 && robot.ColorS.red() <= 100)
        {
            redfound = true;
        }
        if (robot.ColorS.blue() >= 1 && robot.ColorS.blue() <= 100)
        {
            bluefound = true;
        }
        //5)Turn Robot to knock off the correct ball
        if (redfound && ! bluefound)
        {
            while (opModeIsActive() && runtime.milliseconds() < 300)
            {
            }
            robot.leftfront.setPower(0.13);
            robot.rightfront.setPower(-0.13);
            robot.leftback.setPower(0.13);
            robot.rightback.setPower(-0.13);
            sleep(1000);
            robot.ColorS.enableLed(false);
            robot.sideservo.setPosition(1.5);
            while (opModeIsActive() && runtime.milliseconds() < 300)
            {
            }
            robot.leftfront.setPower(-0.13);
            robot.rightfront.setPower(0.13);
            robot.leftback.setPower(-0.13);
            robot.rightback.setPower(0.13);
            sleep(1000);
        }
        else if (bluefound && ! redfound)
        {
            while (opModeIsActive() && runtime.milliseconds() < 300)
            {
            }
            robot.leftfront.setPower(-0.13);
            robot.rightfront.setPower(0.13);
            robot.leftback.setPower(-0.13);
            robot.rightback.setPower(0.13);
            sleep(1000);
            robot.ColorS.enableLed(false);
            robot.sideservo.setPosition(1.5);
            while (opModeIsActive() && runtime.milliseconds() < 300)
            {
            }
            robot.leftfront.setPower(0.13);
            robot.rightfront.setPower(-0.13);
            robot.leftback.setPower(0.13);
            robot.rightback.setPower(-0.13);
            sleep(1000);
        }

        robot.leftfront.setPower(0);
        robot.rightfront.setPower(0);
        robot.leftback.setPower(0);
        robot.rightback.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}