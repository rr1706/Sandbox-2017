package org.usfirst.frc.team1706.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;

import org.usfirst.frc.team1706.robot.subsystems.Feeder;
import org.usfirst.frc.team1706.robot.subsystems.Gear;
import org.usfirst.frc.team1706.robot.subsystems.IMU;
import org.usfirst.frc.team1706.robot.subsystems.Intake;
import org.usfirst.frc.team1706.robot.subsystems.Climber;
import org.usfirst.frc.team1706.robot.subsystems.JetsonServer;
import org.usfirst.frc.team1706.robot.subsystems.Light;
import org.usfirst.frc.team1706.robot.subsystems.LogitechController;
import org.usfirst.frc.team1706.robot.subsystems.PowerPanel;
import org.usfirst.frc.team1706.robot.subsystems.RangeFinder;
import org.usfirst.frc.team1706.robot.subsystems.Shooter;
import org.usfirst.frc.team1706.robot.subsystems.SwerveDrivetrain;
import org.usfirst.frc.team1706.robot.subsystems.SwerveDrivetrain.WheelType;
import org.usfirst.frc.team1706.robot.subsystems.Time;
import org.usfirst.frc.team1706.robot.subsystems.XboxController;
import org.usfirst.frc.team1706.robot.utilities.MathUtils;
import org.usfirst.frc.team1706.robot.utilities.PIDController;
import org.usfirst.frc.team1706.robot.utilities.Vector;
import org.usfirst.frc.team1706.robot.RRLogger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends IterativeRobot {

	SendableChooser<Integer> autoChooser;
	SendableChooser<Integer> boilerSide;
		
	Compressor compressor;
	
	Solenoid side;
	
	double prevGoalTime;

	private int autonomousChoice;
	private int boilerChoice;
	private JetsonServer jet;
	private Thread t;
	private SwerveDrivetrain driveTrain;
	private IMU imu;
	private RRLogger log;

	PWM LED;
	
	double robotOffset;
	
	double previousRCW = 0;

	int disabled = 0;
	
	boolean liftAlignStart = false;
	boolean rotated = false;
	double liftSTR = 0;
	double liftRCW = 0;
	double liftFWD = 0;
	double liftC = 0;
	double oneSkew = 0;
	double oneRotation = 0;
	double oneEnc = 0;
	double frontCenterX = 0;
	double robotCenterX = 0;
	double firstSkew = 0;
	boolean liftDriveStart = false;
	double driveEnc = 0;
	double firstEnc = 0;

	double[][] commands;
	int arrayIndex = -1;
	int autoIntake;
	int autoMove = 0;
	int shootStep = 0;
	double time = 0;
	double autonomousAngle;
	double tSpeed;
	double rSpeed;
	double PIDSpeed;
	double initialPitch;
	double previousDistance;
	double currentDistance;
	double currentTime = 0;
	double prevTimeShoot = 0;
	double prevTimeTap = 0;
	double searchCenter = 22.5;
	boolean shootFlag;
	boolean tap = false;
	boolean goRight = true;
	boolean startSearch = true;
	boolean autoSearch = true;
	boolean sought = false;
	boolean autoShot = false;
	boolean hadVision = false;
	boolean oneRotateDone = false;
	boolean driveDone;
	boolean turnDone;
	boolean timeDone;
	boolean shootDone;
	boolean collisionDone;
	double offsetDeg;
	double prevOffset = 0;
	double prevAngle = 0;
	double oneArm;
	double timeBase;
	boolean timeCheck;
	
	static int robotState;

	double distanceToGoal;

	double armAngleCommand;
	double armSafety = 2.1;

	boolean lowSet = false;
	boolean highSet = false;

	int dx = -1;

	double FWD;
	double STR;
	double RCW;
	
//	double ramp;
//	double rampClock;
//	double rampClock2;
	
	double rampRate = 0;
	double currentRampTime = 0;
	double prevRampTime = 0;

	double keepAngle;
	
	boolean autonomous;
	
	double prevVoltTime = 0;
	double currentVoltTime = 0;

	boolean fieldOriented = true; // start on field orientation
	boolean previousOrientedButton = false;
	boolean currentOrientedButton = false;
	
	boolean slow = false;
	boolean previousSlowButton = false;
	boolean currentSlowButton = false;
	
	boolean shoot = false;
	boolean previousShootButton = false;
	boolean currentShootButton = false;
	
	boolean liftAlign = false;
	boolean currentLiftAlignButton = false;
	boolean previousLiftAlignButton = false;
	
	boolean liftDrive = false;
	boolean currentLiftDriveButton = false;
	boolean previousLiftDriveButton = false;

	boolean currentBoilerAlignButton = false;
	boolean previousBoilerAlignButton = false;
	
	double command = 0;
	boolean firstAlign = true;
	
	PIDController SwerveCompensate;
	PIDController AutoTranslate;
	PIDController GoalAlign;

	double lead;

	double robotRotation;

	double systemTime;
	
	double imuOffset = 0;
	double thing2 = 0;
	
	double armAmperageTrigger = 22.2;
	double armAmperageTriggerTime = 2.2;
	double armAmperageTimer;
	double currentArmAmperage;
	
	double boilerOffset;
	
	double angle;
	
	double gearAngle;
	
	double rotationOffset = 14.1;
	
	Properties application = new Properties();
	File offsets = new File("/home/lvuser/SWERVE_OFFSET.txt");
	
	Vector FRV;
	Vector FLV;
	Vector BRV;
	Vector BLV;
	
	Vector intakeOffset;
	
	public static int getRobotState() {
		return robotState;
	}
	
	public void goalAlign(double error, double distance) {
//		double min;
		
		rotationOffset = SmartDashboard.getNumber("Shooter Rotation Offset", 14.1);
		
		if ((int) boilerSide.getSelected() == 1) {
			error += rotationOffset;
			boilerOffset = -rotationOffset;
		} else if ((int) boilerSide.getSelected() == 2) {
			error -= rotationOffset;
			boilerOffset = rotationOffset;
		}
		
		if (firstAlign) {
			command = MathUtils.resolveDeg(imu.getAngle() + error - robotOffset);
			firstAlign = false;
		}
		SmartDashboard.putNumber("BError", error);
		SmartDashboard.putNumber("BCommand", command);
		SmartDashboard.putBoolean("Using Vision", true);

		System.out.println("com-ang: " + Math.abs(command - imu.getAngle()));
		System.out.println("time: " + Math.abs(prevGoalTime - Time.get()));
		keepAngle = command;
		
		if (Math.abs(command - imu.getAngle()) < 2) {
			if (Math.abs(prevGoalTime - Time.get()) > 1) {
				firstAlign = true;
			}
		} else {
			prevGoalTime = Time.get();
		}
	}
	
	public void liftAlign(double rotation, double skew, double distance) {
		// LABEL Lift Align
		
//		double encDistance = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
		double resolvedAngle = -MathUtils.radToDeg(MathUtils.resolveXrotAngle(MathUtils.degToRad(imu.getAngle())));
		
		if (!liftAlignStart) {
			frontCenterX = distance * Math.cos(MathUtils.degToRad(90 - rotation + skew));
			robotCenterX = Math.sin(MathUtils.degToRad(skew)) * 34/2;
			oneSkew = resolvedAngle + skew;
			oneEnc = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
			firstSkew = -skew;
			liftC = frontCenterX - robotCenterX;
			liftAlignStart = true;
			rotated = false;
			System.out.println(liftC + " | " + rotation + " | " + skew + " | " + distance);
		}
		
//			System.out.println(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getReversed());
//			System.out.println("enc: " + encDistance);
//			System.out.println("lifc: " + liftC);
//			System.out.println("dif: " + Math.abs(encDistance - liftC));
			
		if (rotation > 2.5) {
			liftSTR = -0.165;
		} else if (rotation < -0.5) {
			liftSTR = 0.165;
		} else {
			liftSTR = 0;
			liftAlignStart = false;
			liftAlign = false;
		}
		
//		liftVisionSTR.setInput(rotation);
//		liftVisionSTR.setSetpoint(0);
//		str = lowGearVision.performPID();
		
//		liftVisionRCW.setInput(distance);
//		liftVisionRCW.setSetpoint(0);
//		rcw = liftVisionRCW.performPID();
		
		SmartDashboard.putNumber("RCW", liftRCW);
		SmartDashboard.putNumber("STR", liftSTR);
		SmartDashboard.putNumber("c", liftC);
		
		if (liftAlign && rotation != 9999) { 
			this.STR = liftSTR;
		}
	}
	
	public void liftDrive(double distance) {
		
		double encDistance = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
		
		if (!liftDriveStart) {
			driveEnc = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
			firstEnc = distance;
			liftDriveStart = true;
			System.out.println("1");
		}
		
		if (Math.abs(driveEnc - encDistance) <= firstEnc - 5) {
			this.FWD = -0.3;
			this.STR = 0;
			this.RCW = 0;
			System.out.println("2");
		} else {
			liftDriveStart = false;
			liftDrive = false;
			System.out.println("3");
		}
	}
	
	public void keepAngle() {
		// LABEL keep angle
		
		SwerveCompensate.enable();

		if (XboxController.DPad() != -1) {
			gearAngle = XboxController.DPad();
			
			if ((int) boilerSide.getSelected() == 1 && gearAngle == 180) {
				gearAngle = 121;
			} else if ((int) boilerSide.getSelected() == 2 && gearAngle == 180){
				gearAngle = 241;
			} else if (gearAngle == 0 || gearAngle == 45 || gearAngle == 315) {
				gearAngle = 180; 
			} else if (gearAngle == 90) {
				gearAngle = 120; 
			} else if (gearAngle == 270) {
				gearAngle = 240; 
			} 
			
			keepAngle = gearAngle;
		}
		
		double leadNum = SmartDashboard.getNumber("leadNum", 0);
		lead = RCW * leadNum;

		SmartDashboard.putNumber("DPAD", XboxController.DPad());

		if (Math.abs(RCW) > 0.132 || // If right stick is pressed
				(Math.abs(FWD) < 0.01 && Math.abs(STR) < 0.01) && // If left stick is not pressed
				(XboxController.DPad() == -1) && // If dpad is not pressed
				(XboxController.LTrig() == 0) && // If trigger is not pressed
				(!LogitechController.X()) && // If x is not pressed
				(!autonomous)) { // If teleop

				SwerveCompensate.setPID(0.015, 0.0, 0.0);
				keepAngle = imu.getAngle();

		} else {

			if (LogitechController.X() || XboxController.LTrig() != 0 || thing2 == 1) {
				SwerveCompensate.setPID(0.04, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
				SwerveCompensate.setTolerance(7);
			} else {
				SwerveCompensate.setPID(0.02, 0.0, 0.0);
				SwerveCompensate.setTolerance(12);
			}

			SwerveCompensate.setInput(imu.getAngle());
			SwerveCompensate.setSetpoint(keepAngle);
			
			if (!SwerveCompensate.onTarget()) {
				SwerveCompensate.setPID(0.005, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
			}

			robotRotation = SwerveCompensate.performPID();

			RCW = (robotRotation);

			SmartDashboard.putNumber("Robot Rotation", robotRotation);
		}
	}
	
	/**
	 * Each potentiometer is positioned slightly differently so its initial value is different than the others, so even when the wheels are pointing
	 * straight there are differences.
	 * Proper values may be found and must be calculated for each wheel.
	 */
	public void loadOffsets() {
		// LABEL load offsets

//		Properties application = new Properties();
//		File offsets = new File("/home/lvuser/SWERVE_OFFSET.txt");
//		try {
//			FileInputStream in = new FileInputStream(offsets);
//			application.load(in);
//		} catch (IOException e) {
//			e.printStackTrace();
//		}

		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setOffset(Double.parseDouble(application.getProperty("front_right_offset", "0")));
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setOffset(Double.parseDouble(application.getProperty("front_left_offset", "0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setOffset(Double.parseDouble(application.getProperty("back_left_offset", "0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setOffset(Double.parseDouble(application.getProperty("back_right_offset", "0")));

		SmartDashboard.putNumber("FR offset: ", Double.parseDouble(application.getProperty("front_right_offset", "0")));
		SmartDashboard.putNumber("FL offset: ", Double.parseDouble(application.getProperty("front_left_offset", "0")));
		SmartDashboard.putNumber("BL offset: ", Double.parseDouble(application.getProperty("back_left_offset", "0")));
		SmartDashboard.putNumber("BR offset: ", Double.parseDouble(application.getProperty("back_right_offset", "0")));

		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setPosition(Vector.load(application.getProperty("front_right_pos", "0.0,0.0")));
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setPosition(Vector.load(application.getProperty("front_left_pos", "0.0,0.0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setPosition(Vector.load(application.getProperty("back_left_pos", "0.0,0.0")));
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setPosition(Vector.load(application.getProperty("back_right_pos", "0.0,0.0")));
	}

	public void autonomousAngle(double angle) {
		// LABEL autonomous angle

		SwerveCompensate.setInput(imu.getAngle());
		SwerveCompensate.setSetpoint(angle);
		
		// FIXME try having move to next step immediately, like old code
		SwerveCompensate.setTolerance(7);
		if (!SwerveCompensate.onTarget()) {
			SwerveCompensate.setPID(0.005, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
		} else {
			SwerveCompensate.setPID(0.04, SmartDashboard.getNumber("CompensateI", 0.0), SmartDashboard.getNumber("CompensateD", 0.0));
		}

		robotRotation = SwerveCompensate.performPID();

		RCW = (robotRotation);
	}

	public double turnDrive (double x, double y) {
		double angle = 0;
		
		angle = MathUtils.radToDeg(MathUtils.resolveAngle(Math.atan2(x, y) + Math.PI / 2));
		
		return angle;
	}

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	public void robotInit() {
		// LABEL robot init
		
//		Properties application = new Properties();
//		File offsets = new File("/home/lvuser/SWERVE_OFFSET.txt");
		
		SmartDashboard.putBoolean("Light Bool", true);
		
		SmartDashboard.putNumber("Shooter Rotation Offset", 14.1);
		
		compressor = new Compressor(0);
//		compressor.start();
		
		side = new Solenoid(0);
				
		RangeFinder.start();
		Gear.init();
		
		FRV = Vector.load(application.getProperty("front_right_pos", "0.0,0.0"));
		FLV = Vector.load(application.getProperty("front_left_pos", "0.0,0.0"));
		BLV = Vector.load(application.getProperty("back_left_pos", "0.0,0.0"));
		BRV = Vector.load(application.getProperty("back_right_pos", "0.0,0.0"));
		
		intakeOffset = Vector.load("11.5,0.0");
		
		FRV.add(intakeOffset);
		FLV.add(intakeOffset);
		BLV.add(intakeOffset);
		BRV.add(intakeOffset);
		
		try {
			FileInputStream in = new FileInputStream(offsets);
			application.load(in);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		Time.start();
		
		SmartDashboard.putNumber("RPM", 6000);
		SmartDashboard.putNumber("Encoder RPM", 0);
		SmartDashboard.putNumber("PIDP", 0.15);
		SmartDashboard.putNumber("PIDF", 0.00854);
		SmartDashboard.putNumber("PIDI", 0.0);
		SmartDashboard.putNumber("PIDD", 3.0);
		SmartDashboard.putNumber("Climber Limit", 60);
		SmartDashboard.putNumber("Shooter Offset", 0);
		
		SwerveDrivetrain.loadPorts();
		
		SmartDashboard.putNumber("CompensateP", 0.02);
		SmartDashboard.putNumber("CompensateI", 0.0);
		SmartDashboard.putNumber("CompensateD", 0.0);

//		SmartDashboard.putNumber("lowPcompensate", 0.049);
//		SmartDashboard.putNumber("lowIcompensate", 0.000);
//		SmartDashboard.putNumber("lowDcompensate", 0.000);

		SmartDashboard.putNumber("Autonomous Delay", 0);

		autoChooser = new SendableChooser<Integer>();
		autoChooser.addDefault("Shoot Only", 1);
		autoChooser.addObject("RGear Only", 2);
		autoChooser.addObject("LGear Only", 3);
		autoChooser.addObject("Gear Shoot", 4);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
		
		boilerSide = new SendableChooser<Integer>();
		boilerSide.addDefault("Boiler Right", 1);
		boilerSide.addObject("Boiler Left", 2);
		SmartDashboard.putData("Boiler Side Chooser", boilerSide);

		log = new RRLogger();
		LED = new PWM(9);
		
//		LED.setPeriodMultiplier(PeriodMultiplier.k4X);
		
		try {
			jet = new JetsonServer((short) 5800);
			t = new Thread(jet);
			t.start();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		driveTrain = new SwerveDrivetrain();
		loadOffsets();

		imu = new IMU();
		imu.IMUInit();
		
		SmartDashboard.putNumber("Comp P", 0.015);
		SwerveCompensate = new PIDController(0.015, 0.00, 0.00);
		SwerveCompensate.setContinuous(true);
		SwerveCompensate.setOutputRange(-1.0, 1.0);
		SwerveCompensate.setInputRange(0.0, 360.0);
		SwerveCompensate.setTolerance(1.0);
		
		AutoTranslate = new PIDController(01.000, 0.0, 0.0);
		AutoTranslate.setContinuous(false);
		AutoTranslate.setOutputRange(-1.0, 1.0);
		AutoTranslate.setInputRange(0.0, 250.0);
		
		SmartDashboard.putNumber("GoalP", 0.0145);
		SmartDashboard.putNumber("GoalI", 0.015);
		
		GoalAlign = new PIDController(0.0145, 0.0, 0.0);
		GoalAlign.setContinuous(false);
		GoalAlign.setOutputRange(-1.0, 1.0);
		GoalAlign.setInputRange(-16.0, 16.0);

		SwerveCompensate.enable();
		AutoTranslate.enable();
		GoalAlign.enable();
		
		Light.setSide((int) boilerSide.getSelected());
		System.out.println("Side: " + (int) boilerSide.getSelected());
	}

	public void autonomousInit() {
		// LABEL autonomous init
		jet.setAuto();

		timeCheck = true;
		imu.reset();

		shootFlag = false;

		autonomousChoice = (int) autoChooser.getSelected();
		boilerChoice = (int) boilerSide.getSelected();

		String choice = null;

		if        (autonomousChoice == 1 && boilerChoice == 1) {
			choice = "/home/lvuser/autonomousRShoot.csv";
		} else if (autonomousChoice == 2 && boilerChoice == 1) {
			choice = "/home/lvuser/autonomousRRGear.csv";
		} else if (autonomousChoice == 3 && boilerChoice == 1) {
			choice = "/home/lvuser/autonomousRLGear.csv";
		} else if (autonomousChoice == 4 && boilerChoice == 1) {
//			choice = "/home/lvuser/autonomousRRGearShoot.csv";
			choice = "/home/lvuser/autonomousRRGear.csv";
		} else if (autonomousChoice == 1 && boilerChoice == 2) {
			choice = "/home/lvuser/autonomousLShoot.csv";
		} else if (autonomousChoice == 2 && boilerChoice == 2) {
			choice = "/home/lvuser/autonomousLRGear.csv";
		} else if (autonomousChoice == 3 && boilerChoice == 2) {
			choice = "/home/lvuser/autonomousLLGear.csv";
		} else if (autonomousChoice == 4 && boilerChoice == 2) {
//			choice = "/home/lvuser/autonomousLLGearShoot.csv";
			choice = "/home/lvuser/autonomousLLGear.csv";
		}

		SmartDashboard.putString("Autonomous File", choice);

		time = 0;
		arrayIndex = 0;

		log.start();

		try {
			List<String> lines = Files.readAllLines(Paths.get(choice));
			commands = new double[lines.size()][];
			int l = 0;
			for (String line : lines) {
				String[] parts = line.split(",");
				double[] linel = new double[parts.length];
				int i = 0;
				for (String part : parts) {
					linel[i++] = Double.parseDouble(part);
				}
				commands[l++] = linel;
			}
		} catch (IOException e) {
			e.printStackTrace();
		} catch (NumberFormatException e) {
			System.err.println("Error in configuration!");
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		// LABEL autonomous periodic
		Light.setAuto();
		
		robotState = 0;
		
		side.set(true);
		
		SmartDashboard.putNumber("IMU Angle", imu.getAngle());

		if (timeCheck) {
			timeBase = Time.get();
			System.out.println(timeBase);
			timeCheck = false;
		}
		
		SmartDashboard.putNumber("Elapsed Time", Time.get());

		switch (autoMove) {
			case 0:
				
				driveTrain.drive(new Vector(0, 0), 0);
				if (Time.get() > timeBase + SmartDashboard.getNumber("Autonomous Delay", 0)) {
					autoMove = 1;
				}
				currentDistance = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();
				previousDistance = currentDistance;

				break;
				
			case 1:

				SmartDashboard.putNumber("array Index", arrayIndex);

				CallSmartDashboard.call();
				SmartDashboard.putNumber("IMU Angle", imu.getAngle());

				/*
				 * 0 = translate speed, 1 = rotate speed, 2 = direction to translate, 3 = direction to face,
				 * 4 = distance(in), 5 = shoot, 6 = time out(seconds), 7 = check for collision, 8 = Use Goal Vision, 
				 * 9 = disable feeder, 10 = imu offset, 11 = gear manipulator state, create new
				 */
				currentDistance = SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance();

				tSpeed = commands[arrayIndex][0];
				rSpeed = commands[arrayIndex][1];

				if (commands[arrayIndex][2] != -1) {
					FWD = Math.cos(MathUtils.degToRad(commands[arrayIndex][2]));
					STR = Math.sin(MathUtils.degToRad(commands[arrayIndex][2]));
				} else {
					FWD = 0;
					STR = 0;
				}

				autonomousAngle = commands[arrayIndex][3];

//				if (autonomousAngle != -1) {
//					autonomousAngle(autonomousAngle);
//				}

				if (commands[arrayIndex][4] != -1) {
					AutoTranslate.setInput(Math.abs(currentDistance - previousDistance));
					AutoTranslate.setSetpoint(commands[arrayIndex][4]);
					PIDSpeed = AutoTranslate.performPID();
				}

				SmartDashboard.putNumber("Array Index", arrayIndex);
				SmartDashboard.putNumber("Vel X", imu.getVelocityX());
				SmartDashboard.putNumber("Vel Y", imu.getVelocityY());

				Vector driveCommands;
				driveCommands = MathUtils.convertOrientation(MathUtils.degToRad(imu.getAngle()), FWD, STR);
				FWD = driveCommands.getY() * tSpeed * PIDSpeed;
				STR = driveCommands.getX() * tSpeed * PIDSpeed;
				
				autonomousAngle = commands[arrayIndex][3];
				
				// FIXME
				if (autonomousAngle != -1) {
					autonomousAngle(autonomousAngle);
				}
				
				RCW *= rSpeed;
				
				if ((Math.abs(currentDistance - previousDistance) >= commands[arrayIndex][4]) || commands[arrayIndex][4] == 0) {
					driveDone = true;
					STR = 0;
					FWD = 0;
				}
//				System.out.println("G: " + Math.abs(currentDistance - previousDistance));
//				System.out.println("C: " + commands[arrayIndex][4]);
//				System.out.println("A: " + arrayIndex);
				SmartDashboard.putNumber("Auto Distance Gone", Math.abs(currentDistance - previousDistance));
				SmartDashboard.putNumber("Auto Distance Command", commands[arrayIndex][4]);
				
				SwerveCompensate.setTolerance(1);
				if (SwerveCompensate.onTarget() || commands[arrayIndex][3] == -1) {
					turnDone = true;
					RCW = 0;
				}
				
				if (commands[arrayIndex][5] == 1) {
					Shooter.shoot(jet.getBoilerDistance());
				} else {
					Shooter.stop();
				}
				
				if (Time.get() > timeBase + commands[arrayIndex][6] && commands[arrayIndex][6] != 0) {
					timeDone = true;
					collisionDone = true;
					driveDone = true;
					turnDone = true;
				} else if (commands[arrayIndex][6] == 0) {
					timeDone = true;
				}
				
				if (commands[arrayIndex][7] == 1) {
					if (imu.collisionDetected()) {
						collisionDone = true;
						driveDone = true;
						turnDone = true;
						timeDone = true;
					}
				} else {
					collisionDone = true;
				}
				
				thing2 = commands[arrayIndex][8];
				if (commands[arrayIndex][8] == 1 && jet.getBoilerXrot() != 9999) {
					robotOffset = commands[arrayIndex][12];
					goalAlign(jet.getBoilerXrot(), jet.getBoilerDistance());
					keepAngle();
					autonomous = true;
					System.out.println("BVISION");
				} else {
					SmartDashboard.putBoolean("Using Vision", false);
				}
				
				if (commands[arrayIndex][9] == 1) {
					Shooter.setAutonomous(true);
				} else {
					Shooter.setAutonomous(false);
				}
				
				imuOffset = commands[arrayIndex][10];
				
				Gear.auto(commands[arrayIndex][11]);
				Gear.run();
				
				SmartDashboard.putNumber("Boiler Xrot", jet.getBoilerXrot());
				SmartDashboard.putNumber("Boiler Distance", jet.getBoilerDistance());
				
				SmartDashboard.putNumber("FWD", FWD);
				SmartDashboard.putNumber("STR", STR);
				SmartDashboard.putNumber("RCW", RCW);
				
				driveTrain.drive(new Vector(STR, FWD), RCW);
				
//				System.out.println("Drive: " + driveDone);
//				System.out.println("Turn: " + turnDone);
//				System.out.println("Coll: " + collisionDone);
//				System.out.println("Time: " + timeDone);
				
				if (driveDone && turnDone && collisionDone) {
					arrayIndex++;
					driveDone = false;
					previousDistance = currentDistance;
					turnDone = false;
					timeDone = false;
					collisionDone = false;
					timeBase = Time.get();

				}
				break;
			}
		
		LED.setRaw(Light.getState());
	}

	public void teleopInit() {

		log.start();
		jet.setTeleop();
				
		imu.setOffset(imuOffset);
		
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setID(1);
		SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setID(2);
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setID(4);
		SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setID(3);

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// LABEL teleop periodic
		autonomous = false;
		Light.setTeleop();
		robotState = 1;
		
		if (LogitechController.Y()) {
			side.set(true);
		} else {
			side.set(false);
		}
		
		SmartDashboard.putNumber("Light", LED.getRaw());
		
		SmartDashboard.putNumber("IMU Angle", imu.getAngle());
		
		offsetDeg = MathUtils.radToDeg(MathUtils.resolveXrotAngle(jet.getLiftXrot() - MathUtils.degToRad(-2.6)));
		
		SmartDashboard.putNumber("Range", RangeFinder.getRange());
		
		SmartDashboard.putNumber("Distance to lift", jet.getLiftDistance());
		SmartDashboard.putNumber("Lift Xrot", jet.getLiftXrot());
		SmartDashboard.putNumber("Lift Skew", jet.getLiftSkew());
		
		SmartDashboard.putNumber("Boiler Xrot", jet.getBoilerXrot());
		SmartDashboard.putNumber("Boiler Distance", jet.getBoilerDistance());
					
			// calibration from smartdashboard
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setOffset(SmartDashboard.getNumber("FR offset: ", 0));
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setOffset(SmartDashboard.getNumber("FL offset: ", 0));
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setOffset(SmartDashboard.getNumber("BL offset: ", 0));
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setOffset(SmartDashboard.getNumber("BR offset: ", 0));
		
//			log.newLine();
			log.newPowerLine();
//			log.addPower("1", PowerPanel.a());
//			log.addPower("2", PowerPanel.b());
//			log.addPower("3", PowerPanel.c());
//			log.addPower("4", PowerPanel.d());
//			log.addPower("LED", PowerPanel.e());
//			log.addPower("FL", PowerPanel.f());
//			log.addPower("7", PowerPanel.g());
//			log.addPower("FR", PowerPanel.h());
//			log.addPower("intake", PowerPanel.i());
//			log.addPower("BR", PowerPanel.j());
//			log.addPower("BL", PowerPanel.k());
//			log.addPower("12", PowerPanel.l());
//			log.addPower("13", PowerPanel.m());
//			log.addPower("14", PowerPanel.n());
//			log.addPower("15", PowerPanel.o());
//			log.addPower("16", PowerPanel.p());
			
			log.addPower("BR", PowerPanel.j());
			log.addPower("BL", PowerPanel.k());
			log.addPower("FL", PowerPanel.f());
			log.addPower("FR", PowerPanel.h());
			
			SmartDashboard.putNumber("Distance", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance());
	
			CallSmartDashboard.call();
	
			if (XboxController.Back()) {
				imu.reset(); // robot should be perpendicular to field when pressed.
			}
	
			// forward command (-1.0 to 1.0)
			FWD = -XboxController.LStickY()/10.5*Ds.getBatteryVoltage();
	
			// strafe command (-1.0 to 1.0)
			STR = XboxController.LStickX()/10.5*Ds.getBatteryVoltage();
	
			// rotate clockwise command (-1.0 to 1.0)
			RCW = XboxController.RStickX() * 0.5;
			
	
			// Deadbands for joysticks
			if (Math.abs(XboxController.LStickY()) <= 0.11) {
				FWD = 0.0;
			} else if (FWD > 1.0) {
				FWD = 1.0;
			} else if (FWD < -1.0) {
				FWD = -1.0;
			}
	
			if (Math.abs(XboxController.LStickX()) <= 0.079) {
				STR = 0.0;
			} else if (STR > 1.0) {
				STR = 1.0;
			} else if (STR < -1.0) {
				STR = -1.0;
			}
			
//			if (rampClock == 0.0 && FWD > 0.0){
//				rampClock = Time.get();
//				System.out.println("Time: " + rampClock);
//			} else if(rampClock > 0.0){
//				rampClock2 = Time.get() - rampClock;
//				System.out.println("Calculated Time: " + rampClock);
//				if(rampClock2 > 1.0){
//					rampClock2 = 1.0;
//				}
//				FWD = FWD*Math.pow(rampClock2,3);
//			}
//			System.out.println("Final: " + FWD);
			
			currentRampTime = Time.get();
			if (FWD != 0 || STR != 0 || RCW != 0) {
				if (rampRate < 1) {
					rampRate = Math.abs(currentRampTime - prevRampTime);
				}
				
				FWD *= rampRate;
				STR *= rampRate;
				RCW *= rampRate;
			} else {
				prevRampTime = currentRampTime;
			}
			
			
			if (FWD + STR == 0.0) {
				RCW = XboxController.RStickX();
			}
	
			if (Math.abs(XboxController.RStickX()) <= 0.016) {
				RCW = 0.0;
			}
			
			if (imu.collisionDetected()) {
				XboxController.rumble();
			} else {
				XboxController.stopRumble();
			}
			
			SmartDashboard.putNumber("FWD", FWD);
			SmartDashboard.putNumber("STR", STR);
			SmartDashboard.putNumber("RCW", RCW);
			
//			SmartDashboard.putNumber("Xrot Degrees", MathUtils.radToDeg(jet.getXrot()));
	
//			SmartDashboard.putNumber("Right Stick", XboxController.RStickX());
			SmartDashboard.putNumber("IMU Angle", imu.getAngle());
//			SmartDashboard.putNumber("Keep Angle", keepAngle);
//			SmartDashboard.putNumber("Keep Angle Error", Math.abs(keepAngle - imu.getAngle()));
			
			if (XboxController.B() || LogitechController.B()) {
				if (!XboxController.LB() && !LogitechController.B()) {
					keepAngle = turnDrive(STR, FWD);
				}
				Intake.start();
				
//				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setPosition(FRV);
//				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setPosition(FLV);
//				SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setPosition(BLV);
//				SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setPosition(BRV);
				
			} else if (XboxController.RStickButton()) {
				Intake.reverse();
				
			} else if (!Shooter.getActive()) {
			
				Intake.stop();
				
				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setPosition(Vector.load(application.getProperty("front_right_pos", "0.0,0.0")));
				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setPosition(Vector.load(application.getProperty("front_left_pos", "0.0,0.0")));
				SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setPosition(Vector.load(application.getProperty("back_left_pos", "0.0,0.0")));
				SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setPosition(Vector.load(application.getProperty("back_right_pos", "0.0,0.0")));
			}
	
			double headingDeg = imu.getAngle();
			double headingRad = MathUtils.degToRad(headingDeg);
			
			currentShootButton = LogitechController.A();
			if (currentShootButton && !previousShootButton) {
				shoot = !shoot;

			}
			previousShootButton = currentShootButton;
			
			currentSlowButton = XboxController.X();
			if (currentSlowButton && !previousSlowButton) {
				slow = !slow;

			}
			previousSlowButton = currentSlowButton;
			
			currentVoltTime = Time.get();
			if (Ds.getBatteryVoltage() <= 9.5 && !LogitechController.LTrig()) {
				if (currentVoltTime - prevVoltTime > 0.75) {
					shoot = false;
				}
			} else {
				prevVoltTime = currentVoltTime;
			}
			
			if (shoot) {
				Shooter.shoot(jet.getBoilerDistance());
			} else {
				Shooter.stop();
			}
	
			currentOrientedButton = XboxController.A();
			if (currentOrientedButton && !previousOrientedButton) {
				fieldOriented = !fieldOriented;
	
			}
			previousOrientedButton = currentOrientedButton;
			
			currentLiftAlignButton = XboxController.RB();
			if (currentLiftAlignButton && !previousLiftAlignButton) {
				liftAlign = !liftAlign;
		
			}
			previousLiftAlignButton = currentLiftAlignButton;
			
			currentLiftDriveButton = XboxController.LB();
			if (currentLiftDriveButton && !previousLiftDriveButton) {
//				liftDrive = !liftDrive;
		
			}
			previousLiftDriveButton = currentLiftDriveButton;
			
			boolean boilerReset;
			if (XboxController.LTrig() != 0 || LogitechController.X()) {
				boilerReset = true;
			} else {
				boilerReset = false;
			}
			
			currentBoilerAlignButton = boilerReset;
			if (currentBoilerAlignButton && !previousBoilerAlignButton) {
				firstAlign = true;
				System.out.println("Boiler Reset");
		
			}
			previousBoilerAlignButton = currentBoilerAlignButton;
			
			if (fieldOriented) {
				Vector commands;
				commands = MathUtils.convertOrientation(headingRad, FWD, STR);
				FWD = commands.getY();
				STR = commands.getX();
			} else {
				FWD *= -1;
				STR *= -1;
			}
	
			SmartDashboard.putBoolean("Field Oriented", fieldOriented);
			
			if (liftAlign) {
				liftAlign(jet.getLiftXrot(), jet.getLiftSkew(), jet.getLiftDistance());
			} else {
				liftAlignStart = false;
			}
			
			if (liftDrive) {
				liftDrive(jet.getLiftDistance());
			} else {
				liftDriveStart = false;
			}
			
			if ((XboxController.LTrig() != 0 || LogitechController.X()) && jet.getBoilerDistance() != 9999) {
				goalAlign(jet.getBoilerXrot(), jet.getBoilerDistance());
			} else {
				SmartDashboard.putBoolean("Using Vision", false);
			}
			
			if (XboxController.Y() && RangeFinder.getRange() != -1) {
				if (RangeFinder.getRange() > 7) {
					FWD = -0.18;
				} else if (RangeFinder.getRange() < 4) {
					FWD = 0.18;
				}
			}
			
			GoalAlign.setPID(SmartDashboard.getNumber("GoalP", 0), SmartDashboard.getNumber("GoalI", 0), 0.0);
	
			Climber.run(XboxController.RTrig());
			
			if (LogitechController.RTrig()) {
				Feeder.reverse();
			} else if (!shoot) {
				Feeder.lightStop();
			}
			
			systemTime = Time.get();
	
			SmartDashboard.putNumber("FWD", FWD);
			SmartDashboard.putNumber("STR", STR);
			SmartDashboard.putNumber("RCW", RCW);
	
			if (slow) {
				STR /= 2;
				FWD /= 2;
			}
			boolean boilerOnTarget;
			if (Math.abs(jet.getBoilerXrot() - boilerOffset) < 2) {
				boilerOnTarget = true;
			} else {
				boilerOnTarget = false;
			}
			
			SmartDashboard.putBoolean("Boiler On Target", boilerOnTarget);
			SmartDashboard.putBoolean("Boiler Vision", (jet.getBoilerXrot() != 9999));
			
			SmartDashboard.putNumber("Velocity", IMU.getVelocity());
			keepAngle();
			
			if (LogitechController.RStickX() > 0.2) {
				RCW = 0.18;
			} else if (LogitechController.RStickX() < -0.2) {
				RCW = -0.18;
			}
			
			if (LogitechController.LStickX() > 0.2) {
				STR = 1.0;
			} else if (LogitechController.LStickX() < -0.2) {
				STR = -1.0;
			}
			
			Gear.run();
			
			if (Gear.getTrigger()) {
				FWD = Gear.getFWD();
			}
			driveTrain.drive(new Vector(STR, FWD), RCW); // x = str, y = fwd, rotation = rcw
			
			LED.setRaw(Light.getState());
			
		}

	public void disabledInit() {
		jet.setDisabled();
		shoot = false;
		
		autoMove = 0;
		shootStep = 0;
		
		Light.setDisabled();
		LED.setRaw(Light.getState());
		
		if (disabled < 1) {
			System.out.println("Hello");
			disabled++;
		} else {
			System.out.println("Saving log file(s)");
			log.writeFromQueue();
			
		}
		
	}
	
	public void disabledPeriodic() {
		Light.setDisabled();
		LED.setRaw(Light.getState());
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// LABEL test
		LiveWindow.run();
		
		double speed = (XboxController.RStickX() * 0.3);

		if (XboxController.DPad() != -1) {
			dx = XboxController.DPad();
		}
		
		System.out.println(XboxController.DPad());

		if (dx == 0) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 45) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 90) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 135) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 180) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 225) {
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 270) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(0);
		}

		if (dx == 315) {
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectRotateCommand(speed);

			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectTranslateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).setDirectRotateCommand(0);
			SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).setDirectTranslateCommand(0);
		}
	}
}
