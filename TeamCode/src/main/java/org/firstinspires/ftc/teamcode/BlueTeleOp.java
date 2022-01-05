package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//ignore this

@TeleOp
public class BlueTeleOp extends LinearOpMode {
    public robotInit robot = new robotInit();
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//        boolean spinnyWheely = false;

        boolean freightSnatcher1on = true;
        //boolean freightSnatcher2on = false;

        robot.freightSnatcher1.resetDeviceConfigurationForOpMode();

        waitForStart();

        while (opModeIsActive()) {

            double vertical = 0.55 * (gamepad1.left_stick_y); //move forward, backward
            double turn = 0.55 * (gamepad1.right_stick_x); //turn left, right
            double horizontal = 0.55 * (gamepad1.left_stick_x); //move left, right
            double arm_control = 0.5 * (gamepad1.left_trigger - gamepad1.right_trigger); //arm up,down


            //fudge isn't being used
            //double fudge = 0.25; //TODO decrease speed with fudge factor/power curve so movements are not as jerky

            //Driving for mecanum wheels
            robot.motorFL.setPower(vertical - horizontal - turn);
            robot.motorFR.setPower(vertical + horizontal + turn);
            robot.motorBL.setPower(vertical + horizontal - turn);
            robot.motorBR.setPower(vertical - horizontal + turn);

//            //Driving for rhino (plain, flat) wheels
//            robot.motorFL.setPower(vertical - turn);
//            robot.motorFR.setPower(vertical + turn); //if vertical = 0, turn=-1, then vertical - turn = forward
//            robot.motorBL.setPower(vertical - turn);
//            robot.motorBR.setPower(vertical + turn);


                if(gamepad1.left_bumper) {
                    robot.freightSnatcher1.setPower(-1); //vacuum takes in freight
                }
                else if(gamepad1.right_bumper) { //vacuum spews out freight
                    robot.freightSnatcher1.setPower(1); //vacuum spews out freight
                }
                else {
                    robot.freightSnatcher1.setPower(0.0); //vacuum stops

                }



//            //arm control for elbow and arm lift motors

            robot.elbowMotor.setPower(arm_control);
            robot.armLift.setPower(arm_control);


            if (gamepad1.x) { //spin carousel wheel
                robot.spinnyThing.setPower(0.7);
            }
            else {
                robot.spinnyThing.setPower(0);
            }

        }

//            // Display it for the driver.
//            telemetry.addData("freightSnatcher1", freightSnatcher1on);
//            //telemetry.addData("freightSnatcher2", freightSnatcher2on);
//            telemetry.update();

//           // clamp servo movement
//            if (gamepad1.right_bumper) {
//                //clamps the servo that releases the freight
//                if (freightSnatcher1on) {
//                    robot.freightSnatcher1.setPosition(1);
//                    freightSnatcher1on = false;
//                    telemetry.addLine("servo1 clamped");
//                    telemetry.update();
//                }
//                //releases the servo that clamps the freight
//                else {
//                    robot.freightSnatcher1.setPosition(0.5);
//                    freightSnatcher1on = true;
//                    telemetry.addLine("servo1 released");
//                    telemetry.update();
//                }
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }
//            }



//            // wrist servo movement
//            if (gamepad1.dpad_up) {
//                // wrist servo goes up
//                double newPosition = robot.freightSnatcher2.getPosition() + 0.1;
//                robot.freightSnatcher2.setPosition(newPosition);
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }
//            }
//
//            if (gamepad1.dpad_down) {
//                // wrist servo goes down
//                double newPosition = robot.freightSnatcher2.getPosition() - 0.1;
//                robot.freightSnatcher2.setPosition(newPosition);
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }
            }


//            if (gamepad1.right_bumper) { //clamp freight servo
//                //release the wrist servo
//                if (freightSnatcher2on) {
//                    robot.freightSnatcher2.setPosition(0.5);
//                    freightSnatcher2on = false;
//                    telemetry.addLine("servo2 released");
//                    telemetry.update();
//                }
//                //release the wrist servo
//                else {
//                    robot.freightSnatcher2.setPosition(1);
//                    freightSnatcher2on = true;
//                    telemetry.addLine("servo2 clamped");
//                    telemetry.update();
//                }
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }
//            }



//            //stop intake
//            if(gamepad1.left_bumper){
//                robot.intakeMotor.setPower(0);
//            }
//
//            //start intake
//            if(gamepad1.right_bumper){
//                robot.intakeMotor.setPower(0.9);
//            }

            }
//        telemetry.addData("Status", "Running");
//        telemetry.addLine();
//        telemetry.update();




        /* FUNCTIONS */

//    public void raise(double count) {
//
//        int newElbowMotorTarget;
//
//        // Determine new target position, and pass to motor controller
//        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() + (int)(count);
//        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.elbowMotor.setPower(0.3);
//
//        runtime.reset();
//        while (opModeIsActive() && robot.elbowMotor.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newElbowMotorTarget);
//            telemetry.update();
//        }
//    }

//    public void lower(double count) {
//
//        int newElbowMotorTarget;
//
//        // Determine new target position, and pass to motor controller
//        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() - (int) (count);
//        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.elbowMotor.setPower(0.3);
//
//        runtime.reset();
//        while (opModeIsActive() && robot.elbowMotor.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newElbowMotorTarget);
//            telemetry.update();
//        }
//
//    }