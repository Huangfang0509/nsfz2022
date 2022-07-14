package org.firstinspires.ftc.teamcode.teleop_programes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.HardwareRobot;

@TeleOp(name = "readEncode",group = "Test")
public class readEncode extends OpMode {

    HardwareRobot robot = new HardwareRobot();
    @Override
    public void init() {
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("leftEncode", robot.arm.getCurrentPosition());
        telemetry.addData("rightEncode",robot.rightEncode.getCurrentPosition());
        telemetry.addData("frontEncode",robot.duckRotate.getCurrentPosition());

    }
}
