package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "autoBlue", group = "Competition")
public class autoBlue extends LinearOpMode {

    // Drive motors
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontLeftDrive;

    // Mechanism motors (Control Hub)
    private DcMotor intake;
    private DcMotor outtake;

    // Time constants (adjust through testing)
    private static final double FORWARD_TIME_SECONDS = 0.7;  // Time to move
    private static final double STRAFE_TIME_SECONDS = 0.7;   // Time to strafe

    // Power settings
    private static final double MOVE_POWER = 0.5;
    private static final double STRAFE_POWER = 0.5;
    //private static final double INTAKE_POWER = 1.0;      // Full power clockwise
    //private static final double SHOOTER_POWER = 1.0;     // Full power clockwise

    @Override
    public void runOpMode() {
        // Initialize DRIVE motors
        frontRightDrive = hardwareMap.dcMotor.get("rightFront");
        backRightDrive = hardwareMap.dcMotor.get("rightRear");
        backLeftDrive = hardwareMap.dcMotor.get("leftRear");
        frontLeftDrive = hardwareMap.dcMotor.get("leftFront");

        // Initialize MECHANISM motors
        outtake = hardwareMap.dcMotor.get("OUT");
        intake = hardwareMap.dcMotor.get("IN");

        // Set DRIVE motor directions
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set MECHANISM motor directions for clockwise rotation
        // Adjust if motors spin wrong direction
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to run without encoders
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Method", "Time-Based Auto");
        telemetry.addData("Mechanisms", "Intake & Shooter at FULL power");
        telemetry.addData("Forward Time", "%.1f sec", FORWARD_TIME_SECONDS);
        telemetry.addData("Strafe Time", "%.1f sec", STRAFE_TIME_SECONDS);
        telemetry.update();

        waitForStart();

        // ========== STEP 1: Start Mechanisms ==========
        telemetry.addData("Step", "Starting Mechanisms");
        telemetry.update();

        // Start ALL mechanism motors at FULL POWER clockwise
        //outtake.setPower(0.6);      // Shooting Right - full power
        //intake.setPower(0.6);        // Intake 1 - full power

        sleep(500); // Let mechanisms spin up for 0.5 seconds

        // ========== STEP 2: Move Forward 35 cm ==========
        telemetry.addData("Step", "Moving Forward with Mechanisms");
        telemetry.update();

        // Set all DRIVE motors to move forward
        frontLeftDrive.setPower(MOVE_POWER);
        backLeftDrive.setPower(-MOVE_POWER);
        frontRightDrive.setPower(-MOVE_POWER);
        backRightDrive.setPower(MOVE_POWER);

        // MECHANISMS CONTINUE RUNNING at full power

        // Run for specified time
        sleep((long) (FORWARD_TIME_SECONDS * 1000));

        // Stop DRIVE motors only (keep mechanisms running)
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        sleep(300); // Brief pause before strafing

        // ========== STEP 3: Strafe  ==========
        telemetry.addData("Step", "Strafing ");
        telemetry.update();

        // For strafing LEFT:
        // FL and BR go backward, FR and BL go forward
        //strafe right
        //frontLeftDrive.setPower(STRAFE_POWER);  // Backward
        //backRightDrive.setPower(STRAFE_POWER);  // Backward
        //frontRightDrive.setPower(STRAFE_POWER);   // Forward
        //backLeftDrive.setPower(STRAFE_POWER);   // Forward

        //strafe left
        frontLeftDrive.setPower(-STRAFE_POWER);  // Backward
        backRightDrive.setPower(-STRAFE_POWER);  // Backward
        frontRightDrive.setPower(-STRAFE_POWER);   // Forward
        backLeftDrive.setPower(-STRAFE_POWER);   // Forward

        // MECHANISMS STILL RUNNING at full power

        // Run for specified time
        sleep((long) (STRAFE_TIME_SECONDS * 1000));

        // ========== STEP 4: Stop Everything ==========
        telemetry.addData("Step", "Stopping All Motors");
        telemetry.update();

        // Stop ALL motors
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        outtake.setPower(0);
        intake.setPower(0);

        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.addData("Total Runtime", "%.1f seconds",
                (FORWARD_TIME_SECONDS + STRAFE_TIME_SECONDS + 0.5 + 0.3));
        telemetry.update();

        sleep(2000); // Wait 2 seconds before ending
    }
}
