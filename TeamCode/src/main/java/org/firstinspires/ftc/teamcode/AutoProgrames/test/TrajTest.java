package org.firstinspires.ftc.teamcode.AutoProgrames.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "test")
public class TrajTest extends LinearOpMode {

    SampleMecanumDrive driveBase;
    private Pose2d startPose = new Pose2d(-24,-24,Math.toRadians(-90));






    @Override
    public void runOpMode() throws InterruptedException {
        driveBase = new SampleMecanumDrive(hardwareMap);
        driveBase.setPoseEstimate(startPose);

        Trajectory traj_test1 = driveBase.trajectoryBuilder(startPose,true)
                .lineToSplineHeading(new Pose2d(-24,-48,Math.toRadians(-30))).build();
//        Trajectory ini_step1 =driveBase.trajectoryBuilder(startPose).strafeLeft(25.7).build();


        /** auto drive start **/

        waitForStart();
        if(isStopRequested()) return;

        sleep(1000);

        driveBase.followTrajectory(traj_test1);

        sleep(1000);


    }
}
