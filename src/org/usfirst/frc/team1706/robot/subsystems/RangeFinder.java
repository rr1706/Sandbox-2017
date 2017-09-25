package org.usfirst.frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;

public class RangeFinder {
 
	static I2C range = new I2C(I2C.Port.kOnboard, 0x29);
	static byte[] a = new byte[2];
	static byte[] b = new byte[1];
	static int d;
	static double e;
	
	public static double getRange() {

		range.read(0x1E, 2, a);
		d = (a[0] & 0x0ff) * 256 + (a[1] & 0x0ff);
		e = d / 25.4;
		range.write(0x0B, 0x01);

		if (d <= 20 || d > 2000) {
			e = -1.0;
		}
		
		return e;
	}
	
	public static byte getByte(int x) {
		range.read(x, 1, b);

		return b[0];
	}
	
	public static void start() {
		byte c = getByte(0x89);
		range.write(0x89, c | 0x01);
		range.write(0x88, 0);
		c = getByte(0x60);
		range.write(0x60, c | 0x12);
		range.write(0x01, 0xFF);
		range.write(0x00, 0x02);
		
		System.out.println("Model ID: " + getByte(0xC0));
		
	}
}
