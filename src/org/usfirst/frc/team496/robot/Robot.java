package org.usfirst.frc.team496.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends SampleRobot {
	// RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);
	RobotDrive myRobot = new RobotDrive(0, 1);

	// Talon climbingMotor = new Talon(4);
	// Joystick stick = new Joystick(0);
	XboxController xbox = new XboxController(1);
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();

	PowerDistributionPanel pdp;

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;

	private VisionThread pegVisionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();
	private boolean hasTarget;
	private double prevTurn;
	private double targetDistance;
	private double distance;

	public Robot() {

		pdp = new PowerDistributionPanel();

		HttpCamera camera = CameraServer.getInstance().addAxisCamera("10.4.96.20");
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		camera.setFPS(15);

		CvSink cvSink = CameraServer.getInstance().getVideo();
		CvSource outputStream = CameraServer.getInstance().putVideo("Peg Vision", IMG_WIDTH, IMG_HEIGHT);
		
		pegVisionThread = new VisionThread(camera, new PegPipeline(), pipeline -> {
			Mat source = new Mat();
			cvSink.grabFrame(source);

			if (pipeline.filterContoursOutput().size() > 0) {
				synchronized (imgLock) {
					if(pipeline.filterContoursOutput().size() < 2) {
				Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
				Imgproc.rectangle(source, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height),
						new Scalar(0, 0, 255), 2);
				centerX = (r.x+r.width)/2;
					}
				else {
					Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
					Imgproc.rectangle(source, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height),
							new Scalar(0, 0, 255), 2);
					Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
					
					Imgproc.rectangle(source, new Point(r1.x, r1.y), new Point(r1.x + r1.width, r1.y + r1.height),
							new Scalar(0, 0, 255), 2);
					centerX = (r.x + (r1.x + r1.width)) / 2;
				}
				
				}
				outputStream.putFrame(source);

				/*
					
					if(r1.area()>180){
						targetDistance = r1.x - r.x;
						distance = 0.68 * IMG_WIDTH / (2 * targetDistance * .765);//Math.tan(37.4));
					}
					else{	
						targetDistance = r1.x - r.x;
						distance = 90.0;
					}
					*/
					//targetDistance = r1.x - r.x;
					//distance = 0.68 * IMG_WIDTH / (2 * targetDistance * .765);//Math.tan(37.4));
					//distance = 3;
					hasTarget = true;
					//SmartDashboard.putNumber("Area", r1.area());
					SmartDashboard.putNumber("CenterX", centerX);
					SmartDashboard.putNumber("Distance", distance);
					// System.out.println(centerX);

				
			} else {
				synchronized (imgLock) {
					hasTarget = false;

				}
				outputStream.putFrame(source);
			}

		});
		pegVisionThread.setDaemon(true);
		pegVisionThread.start();

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

		myRobot.setSafetyEnabled(false);
		double centerX;
		double turnRate;
		boolean hasTarget;
		double distance;

		while (isAutonomous() && isEnabled()) {

			synchronized (imgLock) {
				centerX = this.centerX;
				distance = this.targetDistance;
				hasTarget = this.hasTarget;

				// System.out.println(centerX);
			}
			// double distance = Fft*FOVpixel/(2Tpixel*Math.tan(37.4)) ;
			// double distance = 0.68 * IMG_WIDTH / (2 * targetDistance *
			// Math.tan(37.4));
			// double distance = 142.304 * 1/targetDistance;
			distance = distance - 2;

			if (hasTarget) {

				double turn = centerX - (IMG_WIDTH / 2);
				prevTurn = turn;
				if (centerX == 0.0) {
					turnRate = 0.0;
				} else {
					turnRate = -turn * 0.008;
				}

				myRobot.arcadeDrive(-0.8, turnRate);
			} else if (distance >= 0) {
				myRobot.arcadeDrive(0, -prevTurn * .008);
			} else {
				myRobot.arcadeDrive(0, 0);
			}

		}
	}

	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {

		}
	}

	@Override
	public void test() {
		LiveWindow.run();
		System.out.println("Current of 14: " + pdp.getCurrent(14));
	}

}
