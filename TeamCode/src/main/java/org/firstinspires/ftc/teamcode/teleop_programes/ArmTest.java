package org.firstinspires.ftc.teamcode.teleop_programes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareRobot;

@TeleOp(name = "ArmTest",group = "Test")
public class ArmTest extends OpMode {

    HardwareRobot robot = new HardwareRobot();
    private boolean touchArm = false;


    @Override
    public void init() {
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        robot.arm.setPower(0.2);
        if (robot.touchArm.getState()== false){
            touchArm = !touchArm;
        }
        if(touchArm ){
            robot.arm.setPower(0);
            touchArm = false;
        }else if(robot.potentiometerArm.getVoltage() <= 0.1 ){
            robot.arm.setPower(0);
            touchArm = false;
        }

    }
}