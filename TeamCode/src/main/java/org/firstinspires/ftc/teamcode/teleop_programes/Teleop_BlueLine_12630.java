package org.firstinspires.ftc.teamcode.teleop_programes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareRobot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;

@TeleOp(name = "蓝色联盟手动", group = "DriveControl")
public class Teleop_BlueLine_12630 extends OpMode {
    HardwareRobot robot = new HardwareRobot();

    private boolean isSlowly = false;

    private boolean isIntake = false;
//    private boolean isIntakeSwitch = false;

    private boolean isLevel_3 = false;
    private boolean isLevel_2 = false;
    private boolean isLevel_1 = false;
    private boolean isLevel_ini = false;

    private boolean isCatch = false;





    private boolean touchArm = false;
    private boolean touchTurntableLeft = false;
    private boolean touchTurntableRight = false;

    private ElapsedTime keyDelay = new ElapsedTime();
    private ElapsedTime armPidDelay = new ElapsedTime();

//    enum ArmTarget{
//        IntakePos,
//        Level1Pos,
//        Level2Pos,
//        Level3Pos
//    }
//
//    ArmTarget armTarget = ArmTarget.IntakePos;

    @Override
    public void init() {
        robot.init(hardwareMap);
//        robot.Camera.setPosition(0.5);
//        robot.Claw.setPosition(0.5);
//        robot.C_Arm_up.setPosition(0.5);
//        robot.C_Arm_down.setPosition(0.5);
        robot.Led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);


        telemetry.addData("OpMode status","initialized");
        telemetry.update();
        //initialized completed
    }

    @Override
    public void loop() {
        //driveBase
        double drive = -gamepad1.left_stick_y*0.9;
        double turn = -gamepad1.right_stick_x*0.7;
        double translation = gamepad1.left_stick_x*0.8;

        if (gamepad1.left_bumper && keyDelay.seconds() > 0.5) {
            isSlowly = !isSlowly;
            keyDelay.reset();
        }
        if (isSlowly) {
            drive *= 0.5;
            turn *= 0.5;
            translation *= 0.5;
        }

        double[] speed = {
                drive - turn + translation,
                drive - turn - translation,
                drive + turn - translation,
                drive + turn + translation
        };
        double max = 1.0;
        for (double x : speed) {
            if (Math.abs(x) > max) {
                max = Math.abs(x);
            }
        }
        if (max > 1.0) {
            for (int i = 0; i > 3; i++) {
                speed[i] = speed[i] / max;
            }
        }
        robot.leftFront.setPower(speed[0]);
        robot.leftRear.setPower(speed[1]);
        robot.rightFront.setPower(speed[2]);
        robot.rightRear.setPower(speed[3]);

        //intake
        if (gamepad2.left_bumper && keyDelay.seconds() > 0.5) {
            isIntake = !isIntake;
            keyDelay.reset();
        }

//        if (robot.intakeSwitch.getState() == false) {
//            isIntakeSwitch = !isIntakeSwitch;
//        }

        if (gamepad2.right_bumper) {

            robot.intake.setPower(RobotConstants.TeleOp.SPIT_POWER);

        } else if (isIntake ) {
            if(robot.intakeSwitch.getState()){

                robot.intake.setPower(RobotConstants.TeleOp.INTAKE_POWER);

            }else if (!robot.intakeSwitch.getState()) {
                robot.intake.setPower(0);
//                isIntakeSwitch = false;
                isIntake = false;
            }

        }else {
            robot.intake.setPower(0);
        }




        //arm && turntable
        telemetry.addData("potentiometer Votage", robot.potentiometerArm.getVoltage());
        telemetry.addData("tower Pos",robot.turnTable.getCurrentPosition());



        if(gamepad2.y && keyDelay.seconds()>0.5){
            armPidDelay.reset();
            isLevel_3 =true;
            isLevel_2 =false;
            isLevel_1 = false;
            isLevel_ini =false;
            robot.turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(gamepad2.x && keyDelay.seconds()>0.5){
            armPidDelay.reset();
            isLevel_3 =false;
            isLevel_2 =true;
            isLevel_1 = false;
            isLevel_ini =false;
            robot.turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        if(gamepad2.a && keyDelay.seconds()>0.5){
            armPidDelay.reset();
            isLevel_3 =false;
            isLevel_2 =false;
            isLevel_1 = true;
            isLevel_ini =false;
        }

        if(gamepad2.b && keyDelay.seconds()>0.5){
            armPidDelay.reset();
            isLevel_3 =false;
            isLevel_2 =false;
            isLevel_1 = false;
            isLevel_ini =true;
        }


        double Tower_Pos = robot.turnTable.getCurrentPosition();

        if(isLevel_ini ){
            if ( Tower_Pos > -65 && Tower_Pos < 65) {

                robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_INI_POS);

            }else if(armPidDelay.seconds()<2){

                towerPositionPid(0);

            }else {
                robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_INI_POS);
            }

        }else if(isLevel_1 ){
            if(Tower_Pos > -65 && Tower_Pos < 65){

                double u = robot.potentiometerArm.getVoltage();
                if( u < 1.5){
                    robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_1_POS);

                }else {
                    robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_1_POS);

                }
            }else if(armPidDelay.seconds()<2){

                towerPositionPid(0);

            }else {

                double u = robot.potentiometerArm.getVoltage();
                if( u < 1.5){
                    robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_1_POS);

                }else {
                    robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_1_POS);

                }

            }

        }else if(isLevel_2){

            double m = robot.potentiometerArm.getVoltage();
            if( m < 1.1){
                robot.armDownPosPid(RobotConstants.TeleOp.LEVEL_2_POS);

            }else {
                robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_2_POS);

            }

            //turnTable with gamepad2 stick
            double turntablePower = gamepad2.left_stick_x*0.5;

            if(robot.turntableLeft.getState() == false) {
                touchTurntableLeft = true;
            }

            if(robot.turntableRight.getState() == false){
                touchTurntableRight = true;
            }

            if( touchTurntableLeft ){
                robot.turnTable.setPower(Range.clip(turntablePower,0,1));
                touchTurntableLeft = false;
            }else if(touchTurntableRight) {
                robot.turnTable.setPower(Range.clip(turntablePower,-1,0));
                touchTurntableRight = false;
            } else {
                robot.turnTable.setPower(turntablePower);
            }


        }else if (isLevel_3){

            robot.armUpPosPid(RobotConstants.TeleOp.LEVEL_3_POS);

            //turnTable with gamepad2 stick
            double turntablePower = gamepad2.left_stick_x*0.5;

            if(robot.turntableLeft.getState() == false) {
                touchTurntableLeft = true;
            }

            if(robot.turntableRight.getState() == false){
                touchTurntableRight = true;
            }

            if( touchTurntableLeft ){
                robot.turnTable.setPower(Range.clip(turntablePower,0,1));
                touchTurntableLeft = false;
            }else if(touchTurntableRight) {
                robot.turnTable.setPower(Range.clip(turntablePower,-1,0));
                touchTurntableRight = false;
            } else {
                robot.turnTable.setPower(turntablePower);
            }

        }else {
            double armVotage = robot.potentiometerArm.getVoltage();
            if( armVotage < 1.1 && armVotage > 0.5){

                //turnTable with gamepad2 stick

                double turntablePower = gamepad2.left_stick_x*0.5;

                if(robot.turntableLeft.getState() == false) {
                    touchTurntableLeft = true;
                }

                if(robot.turntableRight.getState() == false){
                    touchTurntableRight = true;
                }

                if( touchTurntableLeft ){
                    robot.turnTable.setPower(Range.clip(turntablePower,0,1));
                    touchTurntableLeft = false;
                }else if(touchTurntableRight) {
                    robot.turnTable.setPower(Range.clip(turntablePower,-1,0));
                    touchTurntableRight = false;
                } else {
                    robot.turnTable.setPower(turntablePower);
                }
            }

        }

//    //---------------------Arm for Team Mark--------------------
//        if(gamepad2.dpad_down){
//
//        }

        //---------------------Led--------------------

        if (robot.intakeSwitch.getState()){
            telemetry.addData("intake state is","No");
            robot.Led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        }else if(!robot.intakeSwitch.getState()){
            telemetry.addData("intake state is","yes");
            robot.Led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        //duckRotate
        double duckRotatePower = gamepad2.right_stick_x;
        robot.duckRotate.setPower(duckRotatePower);



    }

    void towerPositionPid(int targetPosition)
    {
        robot.turnTable.setTargetPositionTolerance(20);
        robot.turnTable.setTargetPosition(targetPosition);
        robot.turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turnTable.setPower(0.7);
    }
}