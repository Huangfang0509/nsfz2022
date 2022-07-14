package org.firstinspires.ftc.teamcode.AutoProgrames;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HardwareRobot;

@Autonomous (group = "AutoDriver")
//@Disabled
public class
Red_StorUnit_noCamera_duck_box extends LinearOpMode {
    SampleMecanumDrive driveBase;
    HardwareRobot robot = new HardwareRobot();
    private ElapsedTime m_time = new ElapsedTime();
    //start pose2d
    private Pose2d startPose = new Pose2d(-6.3,-65,Math.toRadians(90));
    //task pose2d
    Pose2d send = new Pose2d(-4,-38,Math.toRadians(30));
    Pose2d p_catch = new Pose2d(37.5,-65.7,Math.toRadians(0));
    Pose2d iniBack = new Pose2d(37.5,65.7,Math.toRadians(-5));
    Pose2d transPose = new Pose2d(10.5,-65.7,Math.toRadians(-5));
    Pose2d duckPosPre = new Pose2d(-55,-58,Math.toRadians(45));

    private double potentiometerVal = 0.0;
    private double armVel =0.0;

    private static final double LEVEL_1_POS = 1.5;
    private static final double LEVEL_2_POS = 1.1;
    private static final double LEVEL_3_POS = 0.6;
    private static final double LEVEL_INI_POS= 2.5;

    @Override
    public void runOpMode() throws InterruptedException {
        driveBase = new SampleMecanumDrive(hardwareMap);
        driveBase.setPoseEstimate(startPose);
        robot.init(hardwareMap);
    //Trajectory build.
        //从机器初始位置到投递点
        Trajectory iniToHub =driveBase.trajectoryBuilder(startPose,true).lineToSplineHeading(send).build();
        //从投递位置到切换点
        Trajectory sendToTrans = driveBase.trajectoryBuilder(send).lineToSplineHeading(transPose).build();
        //从切换点到准备吸取粒子的点，这条路径每个循环会多往前前进一些
        Trajectory moveToP_catch = driveBase.trajectoryBuilder(transPose).lineToLinearHeading(p_catch).build();
        //2英寸每秒的速度往前走3英寸
        Trajectory forwardForIntakeOne = driveBase.trajectoryBuilder(p_catch)
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory forwardForIntakeTwo = driveBase.trajectoryBuilder(p_catch)
                .forward(10,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory forwardForIntakeThree = driveBase.trajectoryBuilder(p_catch)
                .forward(15,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//        //回到准备回头的点
//        Trajectory moveToIniBack = driveBase.trajectoryBuilder(forwardForIntakeOne.end(),true).lineToSplineHeading(iniBack).build();
//        Trajectory moveToIniBack2 = driveBase.trajectoryBuilder(forwardForIntakeTwo.end(),true).lineToSplineHeading(iniBack).build();
        //从回头初始点到切换点
        Trajectory moveToPreSend1 = driveBase.trajectoryBuilder(forwardForIntakeOne.end(),true).lineToSplineHeading(transPose).build();
        Trajectory moveToPreSend2 = driveBase.trajectoryBuilder(forwardForIntakeOne.end(),true).lineToSplineHeading(transPose).build();
        //从切换点到投递点
        Trajectory transToSend = driveBase.trajectoryBuilder(transPose,true).lineToSplineHeading(send).build();
        //从投递点到转小黄鸭预备点
        Trajectory sendToDuckPre =driveBase.trajectoryBuilder(send,true).lineToSplineHeading(duckPosPre).build();
        //慢速倒退
        Trajectory forwardForDuck = driveBase.trajectoryBuilder(duckPosPre)
                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //从鸭子位置到切换点
        Trajectory duckToTrans = driveBase.trajectoryBuilder(forwardForDuck.end()).lineToSplineHeading(transPose).build();

        telemetry.addData("status","initialized");
        telemetry.update();
        //initialized completed

        /** auto drive start **/

        //_______________________________________________________________________________________________
        waitForStart();
        if(isStopRequested())   return;

        //code after START is pressed
       //_______________________________________________________________________________________________



    //step1,投放预载粒子
        driveBase.followTrajectoryAsync(iniToHub);

        while(opModeIsActive() && driveBase.isBusy())
        {
            updateSensorVal();
            armUpPidUpdate(LEVEL_3_POS);
            if(robot.potentiometerArm.getVoltage()<1.1){
                    towerPositionPid(-690);
            }//手臂高于第二层位置时再旋转，防止结构干涉（右侧有一个待定的爪子结构，左侧有一个摄像头结构）
            driveBase.update();
        }

        waitForSpit(LEVEL_3_POS,0.3);//吐出粒子到枢纽

        //---------------------------转鸭子------------------------------------------------------
            //机器开到duckPosPre
            driveBase.followTrajectoryAsync(sendToDuckPre);
            towerPositionPid(0);
            while(opModeIsActive() && driveBase.isBusy())
            {
                armUpPidUpdate(LEVEL_3_POS);
                updateSensorVal();
                driveBase.update();
            }
            //倒退靠近转盘
            driveBase.followTrajectoryAsync(forwardForDuck);
            while(opModeIsActive() && driveBase.isBusy())
            {
                armUpPidUpdate(LEVEL_3_POS);
                updateSensorVal();
                driveBase.update();
            }
            //转鸭子
            waitForDuck(LEVEL_3_POS,2.8);




        //---------------------------第一次回取------------------------------------------------------

        //机器开到切换点
        driveBase.followTrajectoryAsync(duckToTrans);
        towerPositionPid(0);
        while(opModeIsActive() && driveBase.isBusy())
        {
            armUpPidUpdate(LEVEL_3_POS);
            updateSensorVal();
            driveBase.update();
        }

        //机器开到准备取粒子位置
        driveBase.followTrajectoryAsync(moveToP_catch);
        while (opModeIsActive() && driveBase.isBusy()){
            armDownPidUpdate(LEVEL_INI_POS);
            updateSensorVal();
            driveBase.update();
        }

        //边走边取粒子，最终位置计入PoseStore
        driveBase.followTrajectoryAsync(forwardForIntakeTwo);
        while (opModeIsActive() && driveBase.isBusy()){
//                intakeOn();
            armDownPidUpdate(LEVEL_INI_POS);
            updateSensorVal();
//                if(!robot.intakeSwitch.getState()){
//                    intakeOff();
//                }
            driveBase.update();

        }


        //回到缺口外切换点
        driveBase.followTrajectoryAsync(moveToPreSend2);
        while (opModeIsActive()&&driveBase.isBusy()){

            armUpPidUpdate(LEVEL_3_POS);
            updateSensorVal();
            driveBase.update();
        }
        //从切换点回到投递点
        driveBase.followTrajectoryAsync(transToSend);
        towerPositionPid(-690);
        while (opModeIsActive()&&driveBase.isBusy()){

            armUpPidUpdate(LEVEL_3_POS);
            updateSensorVal();
            driveBase.update();
        }
        //投放粒子
        waitForSpit(LEVEL_3_POS,0.3);//吐出粒子到枢纽


        //---------------------------第一次回取------------------------------------------------------

        //----------------------------停靠仓库------------------------------------------------------
        //机器开到切换点
        driveBase.followTrajectoryAsync(sendToTrans);
        towerPositionPid(0);
        while(opModeIsActive() && driveBase.isBusy())
        {
            updateSensorVal();
            armUpPidUpdate(LEVEL_3_POS);
            driveBase.update();
        }

        //机器开到准备取粒子位置
        driveBase.followTrajectoryAsync(moveToP_catch);
        while (opModeIsActive() && driveBase.isBusy()){
            updateSensorVal();
            armDownPidUpdate(LEVEL_INI_POS);
            driveBase.update();
        }



    }//自动执行主体程序结束

    private void armUpPidUpdate(double targetVoltage) {robot.armUpPosPid(targetVoltage);}
    private void armDownPidUpdate(double targetVoltage) {robot.armDownPosPid(targetVoltage);}

    private void updateSensorVal()
    {
        potentiometerVal = robot.potentiometerArm.getVoltage();
        armVel = robot.arm.getVelocity();
    }

    void towerPositionPid(int targetPosition)
    {
        robot.turnTable.setTargetPositionTolerance(20);
        robot.turnTable.setTargetPosition(targetPosition);
        robot.turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turnTable.setPower(0.5);
    }


    private void waitForSpit(double armPos ,double timeout)
    {
        m_time.reset();
        while(opModeIsActive() && m_time.seconds() < timeout)
        {
            robot.intake.setPower(0.8);
            updateSensorVal();
            armUpPidUpdate(armPos);
        }
        robot.disableArm();
        robot.disableIntake();
    }

    private void intakeOn()
    {
        robot.intake.setPower(-0.8);
    }

    private void intakeOff()
    {
        robot.intake.setPower(0);
    }

    private void duckOn()
    {
        robot.duckRotate.setPower(0.8);
    }

    private void duckOff()
    {
        robot.duckRotate.setPower(0);
    }

    private void waitForDuck(double armPos ,double timeout)
    {
        m_time.reset();
        while(opModeIsActive() && m_time.seconds() < timeout)
        {
            robot.duckRotate.setPower(-0.8);
            updateSensorVal();
            armUpPidUpdate(armPos);
        }
        robot.disableArm();
        robot.disableIntake();
    }



}
