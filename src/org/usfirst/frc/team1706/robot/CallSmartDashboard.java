package org.usfirst.frc.team1706.robot;

import org.usfirst.frc.team1706.robot.subsystems.LogitechController;
import org.usfirst.frc.team1706.robot.subsystems.PowerPanel;
import org.usfirst.frc.team1706.robot.subsystems.SwerveDrivetrain;
import org.usfirst.frc.team1706.robot.subsystems.SwerveDrivetrain.WheelType;
import org.usfirst.frc.team1706.robot.subsystems.Time;
import org.usfirst.frc.team1706.robot.subsystems.XboxController;
import org.usfirst.frc.team1706.robot.utilities.MathUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CallSmartDashboard {

	public static void call() {
//		
//		SmartDashboard.putNumber("wheelDistanceAverage", (Math.sqrt(Math.pow((SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getRightSum() + 
//																				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getRightSum() + 
//																				SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getRightSum() + 
//																				SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getRightSum())/4, 2) + 
//																	(Math.pow((SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getForwardSum() + 
//																				SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getForwardSum() + 
//																				SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getForwardSum() + 
//																				SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getForwardSum())/4, 2)))));
//
//		SmartDashboard.putNumber("FL xDelta", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getRightSum()));
//		SmartDashboard.putNumber("FR xDelta", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getRightSum()));
//		SmartDashboard.putNumber("BL xDelta", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getRightSum()));
//		SmartDashboard.putNumber("BR xDelta", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getRightSum()));
//		
//		SmartDashboard.putNumber("FL yDelta", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getForwardSum()));
//		SmartDashboard.putNumber("FR yDelta", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getForwardSum()));
//		SmartDashboard.putNumber("BL yDelta", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getForwardSum()));
//		SmartDashboard.putNumber("BR yDelta", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getForwardSum()));
//		
//		SmartDashboard.putBoolean("FL Reversed", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getReversed()));
//		SmartDashboard.putBoolean("FR Reversed", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getReversed()));
//		SmartDashboard.putBoolean("BL Reversed", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getReversed()));
//		SmartDashboard.putBoolean("BR Reversed", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getReversed()));
//		
//		SmartDashboard.putBoolean("FL MovingRight", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getRight()));
//		SmartDashboard.putBoolean("FR MovingRight", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getRight()));
//		SmartDashboard.putBoolean("BL MovingRight", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getRight()));
//		SmartDashboard.putBoolean("BR MovingRight", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getRight()));
//		
//		SmartDashboard.putBoolean("FL MovingFor", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getFor()));
//		SmartDashboard.putBoolean("FR MovingFor", (SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getFor()));
//		SmartDashboard.putBoolean("BL MovingFor", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getFor()));
//		SmartDashboard.putBoolean("BR MovingFor", (SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getFor()));
//
//		// Robot
//		SmartDashboard.putBoolean("A Button", XboxController.A());
//		SmartDashboard.putBoolean("B Button", XboxController.B());
//		SmartDashboard.putBoolean("X Button", XboxController.X());
//		SmartDashboard.putBoolean("Y Button", XboxController.Y());
//
//		
//		SmartDashboard.putNumber("rawErrorFR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getRawError()));
//		SmartDashboard.putNumber("rawErrorFL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getRawError()));
//		SmartDashboard.putNumber("rawErrorBR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getRawError()));
//		SmartDashboard.putNumber("rawErrorBL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getRawError()));
//		
//		SmartDashboard.putNumber("FL r", MathUtils.radToDeg((SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getrac())));
//		SmartDashboard.putNumber("FR r", MathUtils.radToDeg((SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getrac())));
//		SmartDashboard.putNumber("BL r", MathUtils.radToDeg((SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getrac())));
//		SmartDashboard.putNumber("BR r", MathUtils.radToDeg((SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getrac())));
//		
//		// SwerveDrivetrain
//		 SmartDashboard.putNumber("angleCommandFR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleCommand()));
//		SmartDashboard.putNumber("angleErrorFR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getAngleError()));
//		// SmartDashboard.putNumber("rotationCommandFR", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getWheelRotation());
//		 SmartDashboard.putNumber("speedCommandFR", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getSpeedCommand());
//		 SmartDashboard.putNumber("actualAngleFR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getActualAngle()));
//		// SmartDashboard.putNumber("actualOffsetFR", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getOffset());
//		SmartDashboard.putNumber("wheelDistanceFR", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_RIGHT).getDistance());
//
//		 SmartDashboard.putNumber("angleCommandFL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleCommand()));
//		SmartDashboard.putNumber("angleErrorFL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getAngleError()));
//		// SmartDashboard.putNumber("rotationCommandFL", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getWheelRotation());
//		 SmartDashboard.putNumber("speedCommandFL", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getSpeedCommand());
//		 SmartDashboard.putNumber("actualAngleFL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getActualAngle()));
//		// SmartDashboard.putNumber("actualOffsetFL", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getOffset());
//		SmartDashboard.putNumber("wheelDistanceFL", SwerveDrivetrain.swerveModules.get(WheelType.FRONT_LEFT).getDistance());
//
//		 SmartDashboard.putNumber("angleCommandBL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleCommand()));
//		SmartDashboard.putNumber("angleErrorBL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getAngleError()));
//		// SmartDashboard.putNumber("rotationCommandBL", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getWheelRotation());
//		 SmartDashboard.putNumber("speedCommandBL", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getSpeedCommand());
//		 SmartDashboard.putNumber("actualAngleBL", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getActualAngle()));
//		// SmartDashboard.putNumber("actualOffsetBL", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getOffset());
//		SmartDashboard.putNumber("wheelDistanceBL", SwerveDrivetrain.swerveModules.get(WheelType.BACK_LEFT).getDistance());
//
//		 SmartDashboard.putNumber("angleCommandBR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleCommand()));
//		SmartDashboard.putNumber("angleErrorBR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getAngleError()));
//		// SmartDashboard.putNumber("rotationCommandBR", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getWheelRotation());
//		 SmartDashboard.putNumber("speedCommandBR", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getSpeedCommand());
//		SmartDashboard.putNumber("actualAngleBR", MathUtils.radToDeg(SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getActualAngle()));
//		// SmartDashboard.putNumber("actualOffsetBR", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getOffset());
//		SmartDashboard.putNumber("wheelDistanceBR", SwerveDrivetrain.swerveModules.get(WheelType.BACK_RIGHT).getDistance());
//
//		// Time
//		SmartDashboard.putNumber("Match Time", Time.getMatch());
//		SmartDashboard.putNumber("Match Time Num", Time.getMatch());
	}
}
