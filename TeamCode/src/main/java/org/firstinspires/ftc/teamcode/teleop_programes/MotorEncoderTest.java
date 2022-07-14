package org.firstinspires.ftc.teamcode.teleop_programes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareRobot;

@TeleOp(name = "MotorEncoderTest",group = "Test")
public class MotorEncoderTest extends OpMode {
    HardwareRobot robot = new HardwareRobot();
    private boolean touchArm = false;
    private boolean touchTurntableLeft = false;
    private boolean touchTurntableRight = false;



    @Override
    public void init() {
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        //driveBase Motor test
//        robot.leftFront.setPower(0.5);
//        robot.leftRear.setPower(0.5);
//        robot.rightFront.setPower(0.5);
//        robot.rightRear.setPower(0.5);
//        telemetry.addData("LeftFront position",robot.leftFront.getCurrentPosition());
//        telemetry.addData("LeftRear position",robot.leftRear.getCurrentPosition());
//        telemetry.addData("RightFront position",robot.rightFront.getCurrentPosition());
//        telemetry.addData("RightRear position",robot.rightRear.getCurrentPosition());
//        telemetry.addData("LeftFront velocity",robot.leftFront.getVelocity());
//        telemetry.addData("LeftRear velocity",robot.leftRear.getVelocity());
//        telemetry.addData("RightFront velocity",robot.rightFront.getVelocity());
//        telemetry.addData("RightRear velocity",robot.rightRear.getVelocity());

        //arm Motor test
        double armPower = gamepad1.left_stick_y*0.4;

        if (robot.touchArm.getState()== false){
            touchArm = !touchArm;
        }
        if(touchArm ){
            robot.arm.setPower(Range.clip(armPower,0,1));
            touchArm = false;
        }else if(robot.potentiometerArm.getVoltage() <= 0.1 ){
            robot.arm.setPower(Range.clip(armPower,-1,0));
            touchArm = false;
        }else{
            robot.arm.setPower(armPower);
        }
        telemetry.addData("potentiometerArm Voltage", robot.potentiometerArm.getVoltage());
        telemetry.addData("Arm Velocity",robot.arm.getVelocity());

        //turntable Motor test
        double turntablePower = gamepad1.right_stick_x*0.4;
        telemetry.addData("turn table position",robot.turnTable.getCurrentPosition());

        if(robot.turntableLeft.getState() == false) {
            touchTurntableLeft = !touchTurntableLeft;
        }

        if(robot.turntableRight.getState() == false){
            touchTurntableRight = !touchTurntableRight;
        }

        if( touchTurntableLeft ){
            robot.turnTable.setPower(Range.clip(turntablePower,0,1));
            touchTurntableLeft = false;
            telemetry.addData("turn table position",robot.turnTable.getCurrentPosition());
        }else if(touchTurntableRight) {
            robot.turnTable.setPower(Range.clip(turntablePower,-1,0));
            touchTurntableRight = false;
            telemetry.addData("turn table position",robot.turnTable.getCurrentPosition());
        } else {
            robot.turnTable.setPower(turntablePower);
            telemetry.addData("turn table position",robot.turnTable.getCurrentPosition());
        }

    }
}
