package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
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
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.*;
import org.opencv.imgproc.*;
import com.kauailabs.navx.frc.*;


public class Robot extends SampleRobot implements PIDOutput {
	RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);

	Talon climbingMotor = new Talon(4);
	//Joystick stick = new Joystick(0);
	XboxController xbox = new XboxController(1);
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();
	AHRS ahrs;
	Encoder enc;
	PowerDistributionPanel pdp;

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread pegVisionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();


	

    


	PIDController turnController;
	double rotateToAngleRate;

	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

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

		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH,IMG_HEIGHT);
		
		  pegVisionThread = new VisionThread(camera, new PegPipeline(), pipeline -> {
		        if (!pipeline.filterContoursOutput().isEmpty()) {
		            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
		            synchronized (imgLock) {
		                centerX = r.x + (r.width / 2);
		                
		                //Can do target math here
		            }
		        }
		    });
		    pegVisionThread.start();

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
			
		}
	}

	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {

			boolean rotateToAngle = false;
			if (xbox.getStartButton() == true) {
				ahrs.reset();
			}

			if (xbox.getTrigger(Hand.kRight)) {
				climbingMotor.set(1.0);
			} else if (xbox.getTrigger(Hand.kLeft)) {
				climbingMotor.set(-1.0);
			}

			else if (xbox.getPOV() <= 315 && xbox.getPOV() >= 225) {
				turnController.setSetpoint(0.0f);
				rotateToAngle = true;
			} else if (xbox.getPOV() <= 135 && xbox.getPOV() >= 45) {
				turnController.setSetpoint(90.0f);
				rotateToAngle = true;
			} else if (xbox.getPOV() == 315 || xbox.getPOV() == 0 || xbox.getPOV() == 45) {
				turnController.setSetpoint(179.9f);
				rotateToAngle = true;
			} else if (xbox.getPOV() <= 225 && xbox.getPOV() >= 135) {
				turnController.setSetpoint(-90.0f);
				rotateToAngle = true;
			}

			double currentRotationRate;
			if (rotateToAngle) {
				turnController.enable();
				currentRotationRate = rotateToAngleRate;
			} else {
				turnController.disable();
				currentRotationRate = xbox.getX(Hand.kLeft);
			}

			myRobot.mecanumDrive_Cartesian(-xbox.getX(Hand.kRight), xbox.getY(Hand.kRight), currentRotationRate,
					ahrs.getAngle());

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
