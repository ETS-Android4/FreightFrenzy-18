package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="RedCarousel", group="Pushbot")
public class RedCarousel extends LinearOpMode {

    public robotInit robot = new robotInit();
    ElapsedTime runtime = new ElapsedTime();

    int level; //for auto

    //OpenCvCamera webcam;
    DeterminationPipeline pipeline; //pipeline = series of img coming through camera to process

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status:", "Robot init");
        telemetry.update();
        sleep(1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("CameraMonitorViewID", cameraMonitorViewId);
        telemetry.update();
        sleep(1000);


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        telemetry.addData("Webcam Status", webcamName);
        telemetry.update();
        sleep(1000);

        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        telemetry.addData("Webcam factory", webcam);
        telemetry.update();
        sleep(1000);


        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

//        telemetry.addData("Status:", "WebcamInstance");
//        telemetry.update();
//        sleep(1000);

        pipeline = new DeterminationPipeline();
        webcam.setPipeline(pipeline);

        resetEncoder();
        startEncoderMode();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });


        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            // Telemetry for testing barcode detection
            telemetry.addData("Analysis1", pipeline.getAnalysis1());
            telemetry.addData("Analysis2", pipeline.getAnalysis2());
            telemetry.addData("Analysis3", pipeline.getAnalysis3());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(1000);


            // STEP 1 - Detect element on barcode & store in level var
            if (pipeline.position == webcamtest.DeterminationPipeline.ElementPosition.Level1) {
                telemetry.addData("Detected", "level 1!");
                telemetry.update();

                level = 1;

            } else if (pipeline.position == webcamtest.DeterminationPipeline.ElementPosition.Level2) {
                telemetry.addData("Detected", "level 2!");
                telemetry.update();

                level = 2;

            } else {
                telemetry.addData("Detected", "level 3!");
                telemetry.update();

                level = 3;

            }
            sleep(1000);

//TODO CHANGE tHE ENTIRE THIS THING BELOW :))))))) ASAP :)))))


            //STEP 2 -- go towards carousel wheel
            turnLeft(19);
            //strafeRight(18.6);
            moveBackward(21.5);

            //Touch wheel to carousel in order to spin it
            robot.spinnyThing.setPower(0.7);

            runtime.reset();
            while (runtime.seconds() < 5) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            robot.spinnyThing.setPower(0);

            //STEP 3 -- head to the alliance hub
            strafeRight(7);
            moveBackward(3);
            strafeRight(42.5);
            moveForward(28);

            if (level == 1) {
                telemetry.addData("Detected", "level 1!");
                telemetry.update();

                raise(-90);

            } else if (level == 2) {
                telemetry.addData("Detected", "level 2!");
                telemetry.update();

                raise(-150);

            } else {
                telemetry.addData("Detected", "level 3!");
                raise(-260);

            }

            // place freight on hub
            moveForward(1);
            robot.freightSnatcher1.setPower(-1); //vacuum spews out freight
            runtime.reset();
            while (runtime.seconds() < 3) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            robot.freightSnatcher1.setPower(0); //vacuum stops

            //STEP 4 -- head to storage unit
            moveBackward(37);
            strafeLeft(17.5);
//
        }
    }

    // STEP 1 - Detect where the customized element is placed on the field
    public static class DeterminationPipeline extends OpenCvPipeline {
        public enum ElementPosition {
            Level1,
            Level2,
            Level3
        }

        // Some color constants
        static final Scalar TURQUOISE = new Scalar(163, 210, 202);
        static final Scalar PINK = new Scalar(224, 119, 125);
        static final Scalar LILAC = new Scalar(130, 106, 237);

        // The core values which define the location and size of the sample regions
        static final Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(33, 135);
        static final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(123, 135);
        static final Point BOX3_TOPLEFT_ANCHOR_POINT = new Point(213, 135);

        static final int REGION_WIDTH = 75;
        static final int REGION_HEIGHT = 75;

        Point region1_pointA = new Point(
                BOX1_TOPLEFT_ANCHOR_POINT.x,
                BOX1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                BOX1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                BOX1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region2_pointA = new Point(
                BOX2_TOPLEFT_ANCHOR_POINT.x,
                BOX2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                BOX2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                BOX2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region3_pointA = new Point(
                BOX3_TOPLEFT_ANCHOR_POINT.x,
                BOX3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                BOX3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                BOX3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Creating variables
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        //Box1
        Mat region1_Cb;
        int avg1;

        //Box2
        Mat region2_Cb;
        int avg2;

        //Box3
        Mat region3_Cb;
        int avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile webcamtest.DeterminationPipeline.ElementPosition position = webcamtest.DeterminationPipeline.ElementPosition.Level1;

        // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            //Figure out how much blue is in each region
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    TURQUOISE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    PINK, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    LILAC, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = webcamtest.DeterminationPipeline.ElementPosition.Level1; // Record our analysis
            //find the box/region with maximum red color
            if (avg1 > avg2 && avg1 > avg3) {
                position = webcamtest.DeterminationPipeline.ElementPosition.Level1;
            } else if (avg2 > avg3) {
                position = webcamtest.DeterminationPipeline.ElementPosition.Level2;
            } else {
                position = webcamtest.DeterminationPipeline.ElementPosition.Level3;
            }

            return input;
        }

        //TODO check to see if this works :)
        public int getAnalysis1() {
            return avg1;
        }

        public int getAnalysis2() {
            return avg2;
        }

        public int getAnalysis3() {
            return avg3;
        }
    }




    // FUNCTION TO STRAFE LEFT
    public void strafeLeft(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Strafing left", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }




    // FUNCTION TO STRAFE RIGHT
    public void strafeRight(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Strafing right", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }




    // FUNCTION TO TURN Right
    public void turnRight(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }



    // FUNCTION TO TURN LEFT
    public void turnLeft(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }



    // FUNCTION TO MOVE BACKWARD
    public void moveBackward(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }



    // FUNCTION TO MOVE FORWARD
    public void moveForward(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }



    // ENCODER FUNCTIONS
    public void resetEncoder()
    {
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void startEncoderMode()
    {
        //Set Encoder Mode
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    //RAISE ARM FUNCTION
    public void raise(double count) {

        int Target1;
        int Target2;

        // Determine new target position, and pass to motor controller
        Target1 = robot.elbowMotor.getCurrentPosition() + (int) (count);
        Target2 = robot.armLift.getCurrentPosition() + (int) (count);
        robot.elbowMotor.setTargetPosition(Target1);
        robot.armLift.setTargetPosition(Target2);

        // Turn On RUN_TO_POSITION
        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.armLift.setPower(Math.abs(robot.DRIVE_SPEED));

    }



    //LOWER ARM FUNCTION
    public void lower(double count) {

        int newElbowMotorTarget;

        // Determine new target position, and pass to motor controller
        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() - (int) (count);
        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);

        // Turn On RUN_TO_POSITION
        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));

    }

}