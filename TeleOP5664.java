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

//https://ftc-tricks.com/overview-color-sensor/

//Importing Packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Naming the actual program for the drive station to register when the drivers choose the program
@TeleOp(name = "TeleOP5664", group = "Linear Opmode")
public class TeleOP5664 extends LinearOpMode {
    //Declaring hardware, Cade is our robot's name (not my choice)
    HardwareCade         robot  = new HardwareCade();
    // Establishing all variables for each operation ie:wheels, servos, fronts claw, etc.
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftfront = null;
//    private DcMotor rightfront = null;
//    private DcMotor leftback = null;
//    private DcMotor rightback = null;
//    private DcMotor frontclawlift = null;
//    private Servo leftupservo;
//    private Servo rightupservo;
//    private Servo leftdownservo;
//    private Servo rightdownservo;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (when the driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Setup a variable for each of the drive wheels and the claw in front to save power level for telemetry
            double leftPower;
            double rightPower;
            double liftupPower;
            double backPower;

            //setting which button, trigger, etc. controlling what motor
            rightPower = -gamepad1.right_stick_y *.50;
            leftPower = -gamepad1.left_stick_y *.50;
            liftupPower = gamepad2.left_stick_y;
            backPower = gamepad2.right_stick_y;
            //setting which button, trigger, etc. controlling what servo also, setting position when pressed
            //close top
            if (gamepad2.y)
            {
                robot.rightupservo.setPosition(0.5);
                robot.leftupservo.setPosition(0);
            }

            //open top
            if (gamepad2.b)
            {
                robot.rightupservo.setPosition(0.3);
                robot.leftupservo.setPosition(0.3);
            }

            //close bottom
            if (gamepad2.x)
            {
                robot.rightdownservo.setPosition(0);
                robot.leftdownservo.setPosition(0.7);
            }
            //open bottom
                    if (gamepad2.a)
                    {
                        robot.rightdownservo.setPosition(0.3);
                        robot.leftdownservo.setPosition(0.3);
                    }
            // open/close back
//            if (gamepad1.left_bumper)
//            {
//                robot.backservo.setPosition(0);
//            }
//            // open/close back
//            if (gamepad1.left_bumper)
//            {
//                robot.backservo.setPosition(1.0);
//            }
            // Send calculated power to wheels and front lift
            robot.leftfront.setPower(leftPower);
            robot.rightfront.setPower(rightPower);
            robot.leftback.setPower(leftPower);
            robot.rightback.setPower(rightPower);
            robot.frontclawlift.setPower(liftupPower);
            //robot.backclaw.setPower(backPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
//program by Corbyn LaMar 2017-2018 Team- CyberWolves 5664