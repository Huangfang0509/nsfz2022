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

package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
public class HardwareRobot
{
    /* Public OpMode members. */
    //motor
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear  = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear  = null;

    public DcMotorEx arm = null;
    public DcMotorEx duckRotate =null;
    public DcMotorEx turnTable = null;
    public DcMotorEx rightEncode = null;


    //Servo
    public CRServo intake = null;
    public Servo Claw = null;
    public Servo C_Arm_up = null;
    public Servo C_Arm_down = null;
    public Servo Camera = null;
//    public Servo blinkin = null;
    public RevBlinkinLedDriver Led = null;

    //potentiometer
    public AnalogInput potentiometerArm = null;

    //sensor
    public DigitalChannel touchArm = null;
    public DigitalChannel turntableLeft = null;
    public DigitalChannel turntableRight = null;
    public DigitalChannel intakeSwitch = null;

    private PIDController armUpPid = new PIDController(0.36,0.15,0);  //kp=output/error
    private PIDController armDownPid = new PIDController(0.2,0.05,0);


    HardwareMap hwMap           =  null;
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class,"leftFront");
        leftRear = hwMap.get(DcMotorEx.class,"leftRear");
        rightFront = hwMap.get(DcMotorEx.class,"rightFront");
        rightRear = hwMap.get(DcMotorEx.class,"rightRear");
        arm = hwMap.get(DcMotorEx.class,"arm");
        duckRotate = hwMap.get(DcMotorEx.class,"duckRotate");
        turnTable = hwMap.get(DcMotorEx.class,"turnTable");
        rightEncode = hwMap.get(DcMotorEx.class,"rightEncode");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        duckRotate.setDirection(DcMotorSimple.Direction.FORWARD);
        turnTable.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set all motors to zero power
        arm.setPower(0.0);
        duckRotate.setPower(0.0);
        turnTable.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //initialize All installed servos.
        intake = hwMap.get(CRServo.class,"intake");
        intake.setDirection(CRServo.Direction.FORWARD);

        Claw = hwMap.get(Servo.class,"claw");
        C_Arm_up = hwMap.get(Servo.class,"CArmUp");
        C_Arm_down = hwMap.get(Servo.class,"CArmDown");
        Camera = hwMap.get(Servo.class,"camera");
        Led = hwMap.get(RevBlinkinLedDriver.class,"led");


        //initialize sensor
        potentiometerArm = hwMap.get(AnalogInput.class,"potentiometerArm");

        touchArm = hwMap.get(DigitalChannel.class,"touchArm");
        turntableLeft = hwMap.get(DigitalChannel.class,"turntableLeft");
        turntableRight = hwMap.get(DigitalChannel.class,"turntableRight");
        intakeSwitch = hwMap.get(DigitalChannel.class,"intakeSwitch");
        touchArm.setMode(DigitalChannel.Mode.INPUT);
        turntableLeft.setMode(DigitalChannel.Mode.INPUT);
        turntableRight.setMode(DigitalChannel.Mode.INPUT);
        intakeSwitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public void disableArm(){arm.setMotorDisable();}
    public void disableIntake(){intake.setPower(0);}

    public boolean isTowerReady(double targetPos){
        double pos =turnTable.getCurrentPosition();
        double error = targetPos-pos;
        if(Math.abs(error)<20 ){
            return true;
        }else return false;
    }

    public void armUpPosPid(double targetVol)
    {
        double power  = armUpPid.calculate(potentiometerArm.getVoltage(),targetVol);
        arm.setPower(-power);
    }

    public void armDownPosPid(double targetVol)
    {
        double power  = armDownPid.calculate(potentiometerArm.getVoltage(),targetVol);
        arm.setPower(-power);
    }

 }

