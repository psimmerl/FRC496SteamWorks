package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.*;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot implements PIDOutput {
	RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);
	// Joystick stick = new Joystick(0);
	XboxController xbox = new XboxController(1);
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();
	AHRS ahrs;
	Encoder enc;
	PIDController turnController;
	double rotateToAngleRate;

	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the */
	/* PID Controller will attempt to get. */

	static final double kToleranceDegrees =2.0f;

	public Robot() {
		try {
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		enc = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		myRobot.setExpiration(0.1);
		myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
		myRobot.setInvertedMotor(MotorType.kRearLeft, true);
		;

		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);

	}

	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomous() {
		enc.reset();
		myRobot.setSafetyEnabled(false);
		while (isAutonomous() && isEnabled()) {
			System.out.println(enc.getRaw());
			// System.out.println(enc.)
			// System.out.println(enc.getStopped());
		}
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			// myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(),
			// stick.getTwist(), 0); // drive with arcade style (use right
			// stick)
			// Timer.delay(0.005); // wait for a motor update time

			boolean rotateToAngle = false;
			if (xbox.getStartButton() == true) {
				ahrs.reset();
			}

			else if (xbox.getPOV() <= 315 && xbox.getPOV() >= 225) {
				// myRobot.mecanumDrive_Cartesian(1, 0, 0, ahrs.getAngle());
				turnController.setSetpoint(0.0f);
				rotateToAngle = true;
			} else if (xbox.getPOV() <= 135 && xbox.getPOV() >= 45) {
				// myRobot.mecanumDrive_Cartesian(-1, 0, 0, ahrs.getAngle());
				turnController.setSetpoint(90.0f);
				rotateToAngle = true;
			} else if (xbox.getPOV() == 315 || xbox.getPOV() == 0 || xbox.getPOV() == 45) {
				// myRobot.mecanumDrive_Cartesian(0, 1, 0, ahrs.getAngle());
				turnController.setSetpoint(179.9f);
				rotateToAngle = true;
			} else if (xbox.getPOV() <= 225 && xbox.getPOV() >= 135) {
				// myRobot.mecanumDrive_Cartesian(0, -1, 0, ahrs.getAngle());
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

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
		LiveWindow.run();
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}
