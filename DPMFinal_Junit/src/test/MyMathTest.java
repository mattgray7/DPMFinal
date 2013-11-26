package test;

import static org.junit.Assert.*;

import org.junit.Test;

import Master.MyMath;

public class MyMathTest {
	private static double ERROR = 0.01;
	
	///////////////////////////
	// Test absDifferenceDeg()
	///////////////////////////
	@Test
	public void testAbsDifferenceDegA(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(0.0, 0.0) - 0.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegB(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(0.0, 30.0) - 30.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegC(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(30.0, 0.0) - 30.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegD(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(359.0, 0.0) - 1.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegE(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(0.0, 359.0) - 1.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegF(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(20.0, 350.0) - 30.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegG(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(350.0, 20.0) - 30.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegH(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(181.0, 0.0) - 179.0) < ERROR);
	}
	
	@Test
	public void testAbsDifferenceDegI(){
		assertTrue(Math.abs(MyMath.angleBetweenDeg(269.0, 91.0) - 178.0) < ERROR);
	}
	
	///////////////////////////
	// Test correctionDeg()
	///////////////////////////
	@Test
	public void testCorrectionDegA(){
		assertTrue(Math.abs(MyMath.correctionDeg(0.0, 0.0) - 0.0) < ERROR);
	}

	@Test
	public void testCorrectionDegB(){
		assertTrue(Math.abs(MyMath.correctionDeg(10.0, 0.0) - (-10.0)) < ERROR);
	}

	@Test
	public void testCorrectionDegC(){
		assertTrue(Math.abs(MyMath.correctionDeg(0.0, 10.0) - 10.0) < ERROR);
	}

	@Test
	public void testCorrectionDegD(){
		assertTrue(Math.abs(MyMath.correctionDeg(0.0, 359.0) - (-1.0)) < ERROR);
	}

	@Test
	public void testCorrectionDegE(){
		assertTrue(Math.abs(MyMath.correctionDeg(359.0, 0.0) - 1.0) < ERROR);
	}

	@Test
	public void testCorrectionDegF(){
		assertTrue(Math.abs(MyMath.correctionDeg(170.0, 190.0) - 20.0) < ERROR);
	}

	@Test
	public void testCorrectionDegG(){
		// Either 180.0 or -180.0 is good.
		assertTrue(Math.abs(MyMath.correctionDeg(180.0, 0.0) - 180.0) < ERROR ||
				   Math.abs(MyMath.correctionDeg(180.0, 0.0) - (-180.0)) < ERROR);
	}
	
	/////////////////////////////
	// Test fixAngleDeg()
	/////////////////////////////
	@Test
	public void testFixAngleDegA(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(0.0) - 0.0) < ERROR);
	}

	@Test
	public void testFixAngleDegB(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(360.0) - 0.0) < ERROR);
	}

	@Test
	public void testFixAngleDegC(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(180.0) - 180.0) < ERROR);
	}

	@Test
	public void testFixAngleDegD(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(361.0) - 1.0) < ERROR);
	}

	@Test
	public void testFixAngleDegE(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(721.0) - 1.0) < ERROR);
	}

	@Test
	public void testFixAngleDegF(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(-50.0) - 310.0) < ERROR);
	}

	@Test
	public void testFixAngleDegG(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(-361.0) - 359.0) < ERROR);
	}

	@Test
	public void testFixAngleDegH(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(-721.0) - 359.0) < ERROR);
	}

	@Test
	public void testFixAngleDegI(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(57.0) - 57.0) < ERROR);
	}

	@Test
	public void testFixAngleDegJ(){
		assertTrue(Math.abs(MyMath.fixAngleDeg(325.0) - 325.0) < ERROR);
	}
	
	/////////////////////////////
	// Test averageAngle()
	/////////////////////////////
	@Test
	public void testAverageAngleA(){
		assertTrue(Math.abs(MyMath.averageAngle(0.0, 0.0) - 0.0) < ERROR);
	}

	@Test
	public void testAverageAngleB(){
		assertTrue(Math.abs(MyMath.averageAngle(50.0, 0.0) - 25.0) < ERROR);
	}

	@Test
	public void testAverageAngleC(){
		assertTrue(Math.abs(MyMath.averageAngle(180.0, 0.0) - 90.0) < ERROR ||
				   Math.abs(MyMath.averageAngle(180.0, 0.0) - (270.0)) < ERROR);
	}

	@Test
	public void testAverageAngleD(){
		assertTrue(Math.abs(MyMath.averageAngle(0.0, 180.0) - 90.0) < ERROR ||
				   Math.abs(MyMath.averageAngle(0.0, 180.0) - (270.0)) < ERROR);
	}

	@Test
	public void testAverageAngleE(){
		assertTrue(Math.abs(MyMath.averageAngle(320.0, 40.0) - 0.0) < ERROR);
	}

	@Test
	public void testAverageAngleF(){
		assertTrue(Math.abs(MyMath.averageAngle(40.0, 320.0) - 0.0) < ERROR);
	}

	@Test
	public void testAverageAngleG(){
		assertTrue(Math.abs(MyMath.averageAngle(260.0, 100.0) - 180.0) < ERROR);
	}
}
