package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class teleOp extends LinearOpMode {
    public robotInit robot = new robotInit();
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//        boolean spinnyWheely = false;

        waitForStart();

        while (opModeIsActive()) {

            double vertical = gamepad1.left_stick_y; //move forward, backward
            double turn = gamepad1.right_stick_x; //turn left, right
            double arm_control = gamepad1.left_trigger - gamepad1.right_trigger; //arm up,down
            //fudge isn't being used
            //double fudge = 0.25; //TODO decrease speed with fudge factor/power curve so movements are not as jerky

            //driving for rhino (plain, flat) wheels
            robot.motorFL.setPower(vertical + turn);
            robot.motorFR.setPower(vertical - turn); //if verticle=0, turn=-1, then vertical - turn = forward
            robot.motorBL.setPower(vertical + turn);
            robot.motorBR.setPower(vertical - turn);

            //arm control
            robot.elbowMotor.setPower(arm_control);

            if (gamepad1.left_bumper){ //release freight servo
                robot.freightSnatcher1.setPosition(1);
                robot.freightSnatcher2.setPosition(1);

            }
            if (gamepad1.right_bumper) { //clamp freight servo
                robot.freightSnatcher1.setPosition(0.5);
                robot.freightSnatcher2.setPosition(0.5);
            }


            if(gamepad1.y){ //spin carousel wheel
                if(robot.spinnyThing.getPower() == 0){
                    //start wheel
                    robot.spinnyThing.setPower(0.2);
                }
                if(robot.spinnyThing.getPower() != 0){
                    robot.spinnyThing.setPower(0);
                }

            }




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
    }




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
}
