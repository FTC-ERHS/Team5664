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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareCade
{
    /* Public OpMode members. */
    public DcMotor leftfront = null;
    public DcMotor rightfront = null;
    public DcMotor leftback = null;
    public DcMotor rightback = null;
    public DcMotor frontclawlift = null;
    //public DcMotor backclaw = null;
    public Servo leftupservo;
    public Servo rightupservo;
    public Servo leftdownservo;
    public Servo rightdownservo;
    public Servo sideservo;
    //public Servo backservo;
    public ModernRoboticsI2cColorSensor ColorS;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */
    public HardwareCade(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors/Servos/Sensors
        leftfront = hwMap.get(DcMotor.class, "leftfront");
        rightfront = hwMap.get(DcMotor.class, "rightfront");
        leftback = hwMap.get(DcMotor.class, "leftback");
        rightback = hwMap.get(DcMotor.class, "rightback");
        frontclawlift = hwMap.get(DcMotor.class, "frontclawlift");
        leftupservo = hwMap.get(Servo.class, "leftupservo");
        rightupservo = hwMap.get(Servo.class, "rightupservo");
        leftdownservo = hwMap.get(Servo.class, "leftdownservo");
        rightdownservo = hwMap.get(Servo.class, "rightdownservo");
        ColorS = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("ColorS");
        sideservo = hwMap.get (Servo.class, "sideservo");
        //backclaw = hwMap.get (DcMotor.class, "backclaw");
        //backservo = hwMap.get (Servo.class, "backservo");

        //Set Direction of Motors
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        frontclawlift.setDirection(DcMotor.Direction.FORWARD);
        //backclaw.setDirection(DcMotor.Direction.FORWARD);

        //Define Initialization Positions and put off color sensor until needed (Works for both TeleOp and Autonomous)
        leftupservo.setPosition(1);
        rightupservo.setPosition(0);
        rightdownservo.setPosition(1);
        leftdownservo.setPosition(0);

        ColorS.enableLed(false);

        sideservo.setPosition(1.5);
    }
 }

