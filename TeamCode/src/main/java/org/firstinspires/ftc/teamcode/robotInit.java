package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;



//@Disabled
public class robotInit {

    /* Public OpMode members. */
    //creating objects
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor elbowMotor; //raise/lower arm
    public DcMotor spinnyThing; //carousel motor
    public DcMotor armLift; //arm lifting mechanism
    //public DcMotor intakeMotor;

    public CRServo freightSnatcher1; //freight claw 1
    //public Servo freightSnatcher2; //freight claw 2
    //public Servo ringFlicker; // potential carousel spinner

    //from Encoder Sample
    double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;  // For figuring circumference
    double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    double     DRIVE_SPEED             = 0.45;
    double     teleOP_FORWARD_SPEED    = 1;

    /* local OpMode members. */
    HardwareMap hardwareMap ;

    // instantiate = to create an instance of = constructor
    /* Constructor */
    public robotInit(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;



        // Define and Initialize Motors
        motorFL = hardwareMap.get(DcMotor.class, "motor_fl");
        motorFR = hardwareMap.get(DcMotor.class, "motor_fr");
        motorBL = hardwareMap.get(DcMotor.class, "motor_bl");
        motorBR = hardwareMap.get(DcMotor.class, "motor_br");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
        spinnyThing = hardwareMap.get(DcMotor.class, "spinnyThing");
        armLift = hardwareMap.get(DcMotor.class, "armLift");

        // Set the direction of the DC motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);
        spinnyThing.setDirection(DcMotor.Direction.FORWARD);
        armLift.setDirection(DcMotor.Direction.FORWARD);

        // Set all DC motors to zero power
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
        elbowMotor.setPower(0);
        spinnyThing.setPower(0);
        armLift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnyThing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        freightSnatcher1 = hardwareMap.get(CRServo.class, "freightSnatcher1");
        //freightSnatcher2 = hardwareMap.get(Servo.class, "freightSnatcher2");


        //init servos
        //freightSnatcher1.setPosition(0.72);
        //freightSnatcher2.setPosition(0.4);

    }
}