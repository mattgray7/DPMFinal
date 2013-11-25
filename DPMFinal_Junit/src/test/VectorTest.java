package test;

import static org.junit.Assert.*;
import org.junit.Test;

import Master.Vector;


public class VectorTest {
	
	///////////////////////
	// Getters / setters
	///////////////////////
	@Test
	public void testGetX() {
		Vector v = new Vector(10.0, 20.0, 30.0);
		assertTrue(v.getX() == 10.0);
	}

	@Test
	public void testGetY() {
		Vector v = new Vector(10.0, 20.0, 30.0);
		assertTrue(v.getY() == 20.0);
	}

	@Test
	public void testGetZ() {
		Vector v = new Vector(10.0, 20.0, 30.0);
		assertTrue(v.getZ() == 30.0);
	}
	

	///////////////////////
	// Divide
	///////////////////////
	@Test
	public void testDivideA(){
		Vector v = new Vector(10.0, 20.0, 30.0);
		v.divide(1.0);
		
		assertTrue(v.getX() == 10.0 &&
				   v.getY() == 20.0 &&
				   v.getZ() == 30.0);
	}
	
	@Test
	public void testDivideB(){
		Vector v = new Vector(10.0, 20.0, 30.0);
		v.divide(-1.0);
		
		assertTrue(v.getX() == -10.0 &&
				   v.getY() == -20.0 &&
				   v.getZ() == -30.0);
	}
	
	@Test
	public void testDivideC(){
		Vector v = new Vector(10.0, 20.0, 30.0);
		v.divide(5.0);
		
		assertTrue(v.getX() == 2.0 &&
				   v.getY() == 4.0 &&
				   v.getZ() == 6.0);
	}
	
	@Test
	public void testDivideD(){
		Vector v = new Vector(10.0, 20.0, 30.0);
		v.divide(-5.0);
		
		assertTrue(v.getX() == -2.0 &&
				   v.getY() == -4.0 &&
				   v.getZ() == -6.0);
	}
	

	///////////////////////
	// Add
	///////////////////////
	@Test
	public void testAddA(){
		Vector a = new Vector(0.0, 0.0, 0.0);
		Vector b = new Vector(0.0, 0.0, 0.0);
		Vector sum = Vector.add(a, b);
		
		assertTrue(sum.getX() == 0.0 && sum.getY() == 0.0 && sum.getZ() == 0.0);
	}
	
	@Test
	public void testAddB(){
		Vector a = new Vector(1.0, 1.0, 1.0);
		Vector b = new Vector(0.0, 0.0, 0.0);
		Vector sum = Vector.add(a, b);
		
		assertTrue(sum.getX() == 1.0 && sum.getY() == 1.0 && sum.getZ() == 1.0);
	}

	@Test
	public void testAddC(){
		Vector a = new Vector(10.0, 20.0, 30.0);
		Vector b = new Vector(5.0, 15.0, 25.0);
		Vector sum = Vector.add(a, b);
		
		assertTrue(sum.getX() == 15.0 && sum.getY() == 35.0 && sum.getZ() == 55.0);
	}

	@Test
	public void testAddD(){
		Vector a = new Vector(0.0, 0.0, 0.0);
		Vector b = new Vector(-5.0, -10.0, -15.0);
		Vector sum = Vector.add(a, b);
		
		assertTrue(sum.getX() == -5.0 && sum.getY() == -10.0 && sum.getZ() == -15.0);
	}

	@Test
	public void testAddE(){
		Vector a = new Vector(-10.0, -20.0, -30.0);
		Vector b = new Vector(-5.0, -15.0, -25.0);
		Vector sum = Vector.add(a, b);
		
		assertTrue(sum.getX() == -15.0 && sum.getY() == -35.0 && sum.getZ() == -55.0);
	}

	@Test
	public void testAddF(){
		Vector a = new Vector(-10.0, -10.0, -10.0);
		Vector b = new Vector(10.0, 10.0, 10.0);
		Vector sum = Vector.add(a, b);
		
		assertTrue(sum.getX() == 0.0 && sum.getY() == 0.0 && sum.getZ() == 0.0);
	}
	

	///////////////////////
	// Average
	///////////////////////
	@Test
	public void testAverageA(){
		Vector a = new Vector(0.0, 0.0, 0.0);
		Vector b = new Vector(0.0, 0.0, 0.0);
		Vector vectorList[] = {a, b};
		Vector average = Vector.average(vectorList);
		
		assertTrue(average.getX() == 0.0 && average.getY() == 0.0 && average.getZ() == 0.0);
	}

	@Test
	public void testAverageB(){
		Vector a = new Vector(1.0, 1.0, 1.0);
		Vector b = new Vector(0.0, 0.0, 0.0);
		Vector vectorList[] = {a, b};
		Vector average = Vector.average(vectorList);
		
		assertTrue(average.getX() == 0.5 && average.getY() == 0.5 && average.getZ() == 0.5);
	}

	@Test
	public void testAverageC(){
		Vector a = new Vector(1.0, 1.0, 1.0);
		Vector b = new Vector(-1.0, -1.0, -1.0);
		Vector vectorList[] = {a, b};
		Vector average = Vector.average(vectorList);
		
		assertTrue(average.getX() == 0.0 && average.getY() == 0.0 && average.getZ() == 0.0);
	}

	@Test
	public void testAverageD(){
		Vector a = new Vector(15.0, 1.0, 12.0);
		Vector b = new Vector(5.0, 45.0, 13.0);
		Vector vectorList[] = {a, b};
		Vector average = Vector.average(vectorList);
		
		assertTrue(average.getX() == 10.0 && average.getY() == 23.0 && average.getZ() == 12.5);
	}

	@Test
	public void testAverageE(){
		Vector a = new Vector(1.0, 2.0, 3.0);
		Vector b = new Vector(4.0, 5.0, 6.0);
		Vector c = new Vector(7.0, 8.0, 9.0);
		Vector d = new Vector(10.0, 11.0, 12.0);
		Vector vectorList[] = {a, b, c, d};
		Vector average = Vector.average(vectorList);
		
		assertTrue(average.getX() == 5.5 && average.getY() == 6.5 && average.getZ() == 7.5);
	}
}
