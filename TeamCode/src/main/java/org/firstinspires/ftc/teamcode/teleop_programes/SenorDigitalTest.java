package org.firstinspires.ftc.teamcode.teleop_programes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.HardwareRobot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;

@TeleOp(name = "SenorDigitalTest",group = "Test")
public class SenorDigitalTest extends OpMode {
    HardwareRobot robot = new HardwareRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("potentiometer Votage", robot.potentiometerArm.getVoltage());

    }
    @Override
    public void loop() {
//        if(robot.touchArm.getState()== true){
//            telemetry.addData("touchArm", "Is Not Pressed");
//        }else{
//            telemetry.addData("touchArm", "Is Pressed");
//        }
//        if(robot.turntableLeft.getState()== true){
//            telemetry.addData("turntableLeft ","Is Not Pressed");
//        }else {
//            telemetry.addData("turntableLeft", "Is Pressed");
//        }
//        if(robot.turntableRight.getState()== true){
//            telemetry.addData("turntableRight ","Is Not Pressed");
//        }else {
//            telemetry.addData("turntableRight", "Is Pressed");
//        }

        telemetry.addData("potentiometer Votage", robot.potentiometerArm.getVoltage());

        //armup
        if(gamepad2.b){
            robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_INI_POS);
            telemetry.addData("arm power",robot.arm.getPower());
        }else if(gamepad2.a){
            robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_1_POS);
            telemetry.addData("arm power",robot.arm.getPower());
        }else if(gamepad2.x){
            robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_2_POS);
            telemetry.addData("arm power",robot.arm.getPower());
        }else if(gamepad2.y){
            robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_3_POS);
            telemetry.addData("arm power",robot.arm.getPower());
        }


//        //armdown
//        if(gamepad2.b){
//            robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_INI_POS);
//            telemetry.addData("arm power",robot.arm.getPower());
//        }else if(gamepad2.a){
//            robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_1_POS);
//            telemetry.addData("arm power",robot.arm.getPower());
//        }else if(gamepad2.x){
//            robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_2_POS);
//            telemetry.addData("arm power",robot.arm.getPower());
//        }else if(gamepad2.y){
//            robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_3_POS);
//            telemetry.addData("arm power",robot.arm.getPower());
//        }

        if(robot.intakeSwitch.getState() ){
            telemetry.addData("intake state", "NO");
        }else {
            telemetry.addData("intake state" ,"YES");
        }


    }
}
