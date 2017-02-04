package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import com.kauailabs.navx.frc.*;

public class Robot extends SampleRobot implements PIDOutput {
	RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);

	Victor climbingMotor = new Victor(4);
	//Joystick stick = new Joystick(0);
	XboxController xbox = new XboxController(1);
	XboxController opXbox = new XboxController(0);
	
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();
	AHRS ahrs;
	Encoder enc;
	PowerDistributionPanel pdp;

	/*Thread visionThread = new Thread(() -> {
		// Get the Axis camera from CameraServer
		AxisCamera camera = CameraServer.getInstance().addAxisCamera("axis-camera.local");
		// Set the resolution
		camera.setResolution(640, 480);

		// Get a CvSink. This will capture Mats from the camera
		CvSink cvSink = CameraServer.getInstance().getVideo();
		// Setup a CvSource. This will send images back to the Dashboard
		CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

		// Mats are very memory expensive. Lets reuse this Mat.
		Mat mat = new Mat();

		// This cannot be 'true'. The program will never exit if it is. This
		// lets the robot stop this thread when restarting robot code or
		// deploying.
		while (!Thread.interrupted()) {
			// Tell the CvSink to grab a frame from the camera and put it
			// in the source mat. If there is an error notify the output.
			if (cvSink.grabFrame(mat) == 0) {
				// Send the output the error.
				outputStream.notifyError(cvSink.getError());
				// skip the rest of the current iteration
				continue;
			}
			// Put a rectangle on the image
			Imgproc.rectangle(mat, new Point(270, 290), new Point(370, 190), new Scalar(0, 255, 75), 1);
			// Give the output stream a new image to display
			outputStream.putFrame(mat);
		}
	});*/

	PIDController turnController;
	double rotateToAngleRate;
	boolean changed;
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

	
	static double timeStart = Timer.getFPGATimestamp();
	static double lastModeSwitchTime = timeStart;
	static int xMultiplier = -1;
	static int yMultiplier = 1;
	
	
	
	/* This tuning parameter indicates how close to "on target" the */
	/* PID Controller will attempt to get. */

	static final double kToleranceDegrees = 1.0f;

	public Robot() {

		try {
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		pdp = new PowerDistributionPanel();
		enc = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		myRobot.setExpiration(0.1);
		myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
		myRobot.setInvertedMotor(MotorType.kRearLeft, true);

		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

	//	visionThread.setDaemon(true);
	//	visionThread.start();

		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		LiveWindow.addSensor("PowerSystem", "Current", pdp);

		LiveWindow.run();

	}

	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
	}

	@Override
	public void autonomous() {
		enc.reset();
		myRobot.setSafetyEnabled(false);
		while (isAutonomous() && isEnabled()) {
			// System.out.println(enc.getRaw());
		}
	}

	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			
			boolean rotateToAngle = false;
			if (xbox.getAButton() == true) {
				ahrs.reset();
			}
			if (opXbox.getRawButton(2)) {
				climbingMotor.set(1.0);
			} else if (opXbox.getRawButton(3)) {
				climbingMotor.set(-1.0);
			}else{
				climbingMotor.set(0.0);
			}
			
			double currentRotationRate;
			if (rotateToAngle) {
				turnController.enable();
				currentRotationRate = rotateToAngleRate;
			} else {
				turnController.disable();
				currentRotationRate = (xbox.getX(Hand.kLeft))/2;
			}
			
			if(currentRotationRate == 0 && changed){
				turnController.enable();
				turnController.setSetpoint(ahrs.getAngle());
				currentRotationRate = rotateToAngleRate;
				changed = !changed;
			}else{
				changed = true;
			}

			/*if (xbox.getPOV() == 0) {
				myRobot.mecanumDrive_Cartesian(0, -1, currentRotationRate,
						0);
			} else if (xbox.getPOV() == 45) {
				myRobot.mecanumDrive_Cartesian(1, -1, currentRotationRate,
						0);				
			} else if (xbox.getPOV() == 90) {
				myRobot.mecanumDrive_Cartesian(0, 1, currentRotationRate,
						0);
			} else if (xbox.getPOV() == 135) {
				myRobot.mecanumDrive_Cartesian(-1, -1, currentRotationRate,
						0);
			} else if (xbox.getPOV() == 180) {
				myRobot.mecanumDrive_Cartesian(0, 1, currentRotationRate,
						0);
			} else if (xbox.getPOV() == 225) {
				myRobot.mecanumDrive_Cartesian(-1, 1, currentRotationRate,
						0);
			} else if (xbox.getPOV() == 270) {
				myRobot.mecanumDrive_Cartesian(1, 0, currentRotationRate,
						0);
			} else if (xbox.getPOV() == 315) {
				myRobot.mecanumDrive_Cartesian(1, 1, currentRotationRate,
						0);
			}*/
			
			if(xbox.getRawButton(6) && (Timer.getFPGATimestamp()-lastModeSwitchTime) > .3){
				xMultiplier *=-1;
				yMultiplier *=-1;
				lastModeSwitchTime = Timer.getFPGATimestamp();
			} else if(xbox.getRawButton(6)){
				lastModeSwitchTime = Timer.getFPGATimestamp();
			}
			
			if(xbox.getX(Hand.kRight) != 0 || xbox.getY(Hand.kRight) != 0 || xbox.getX(Hand.kLeft) != 0){
				myRobot.mecanumDrive_Cartesian(xMultiplier*xbox.getX(Hand.kRight), yMultiplier*xbox.getY(Hand.kRight), currentRotationRate,
					0);
			}
			Timer.delay(.00025);

		}

	}

	@Override
	public void test() {
		LiveWindow.run();
		System.out.println("Current of 14: " + pdp.getCurrent(14));
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}
