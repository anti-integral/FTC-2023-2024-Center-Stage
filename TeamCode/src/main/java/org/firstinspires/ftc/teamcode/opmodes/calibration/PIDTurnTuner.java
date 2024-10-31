package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequence;
import org.firstinspires.ftc.teamcode.subsystems.movement.MovementSequenceBuilder;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;

// FTC DASHBOARD
// USAGE:
//  1) Connect to the Control Hub's wifi
//  2) Navigate to "192.168.43.1:8080/dash" using a web browser
//
// TODO: Camera stream monitor

@Config
@TeleOp(name = "Drivetrain Turn PID Tuner")
public class PIDTurnTuner extends LinearOpMode {
    public static double turnDegrees = 180;
    final private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot hardwareRobot;
    private static DriveSubsystem drive;
    private MultipleTelemetry telemetry = new MultipleTelemetry(new TelemetryImpl((OpMode) this), FtcDashboard.getInstance().getTelemetry());


    @Override
    // The "Main" code will go in here
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
        drive = robot.getDrive();
        runtime.reset();

        waitForStart();

        // begin tuning sequence
        while (opModeIsActive()) {

            MovementSequence forward = new MovementSequenceBuilder()
                    .turnRight(turnDegrees)
                    .build();

            // forward 1000√2 ticks
//            drive.driveByAngularEncoder(SpeedCoefficients.getAutonomousDriveSpeed(), 3000, 3000, Math.PI/2);
            drive.followMovementSequence(forward);
            while (opModeIsActive() && !gamepad1.triangle);
            // backward 1000√2 ticks

            MovementSequence backward = new MovementSequenceBuilder()
                    .turnLeft(turnDegrees)
                    .build();
//            drive.driveByAngularEncoder(SpeedCoefficients.getAutonomousDriveSpeed(), 3000, 3000, -Math.PI/2);
            drive.followMovementSequence(backward);
            while (opModeIsActive() && !gamepad1.triangle);
        }

        drive.stopController();
    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < ms));
    }
}
