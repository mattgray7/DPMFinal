package test;

import static org.junit.Assert.*;

import org.junit.Test;

import Master.SmoothDifferenceFilter;


public class SmoothDifferenceFilterTest {

	////////////////////////////
	// Test that the behavior from Filter is conserved.
	////////////////////////////
	@Test
	public void testFilterA() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		assertTrue(f.getFilteredValue() == 0);
	}

	@Test
	public void testFilterB() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		f.add(1);
		
		assertTrue(f.getFilteredValue() == 1);
	}

	@Test
	public void testFilterC() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		f.add(1);
		f.add(2);
		
		assertTrue(f.getFilteredValue() == 2);
	}

	/////////////////////////////
	// Test that it returns the smooth difference.
	/////////////////////////////
	@Test
	public void testSmoothDifferenceA() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		f.add(1);
		f.add(12);
		f.add(3);
		f.add(4);
		
		assertTrue(f.getFilteredValue() == -3);
	}

	@Test
	public void testSmoothDifferenceB() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		f.add(1);
		f.add(12);
		f.add(100);
		f.add(90);
		
		assertTrue(f.getFilteredValue() == 88);
	}

	@Test
	public void testSmoothDifferenceC() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		f.add(5);
		f.add(12);
		f.add(3);
		f.add(-14);
		
		assertTrue(f.getFilteredValue() == -14);
	}

	@Test
	public void testSmoothDifferenceD() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		f.add(1);
		f.add(-1);
		f.add(-4);
		f.add(-4);
		
		assertTrue(f.getFilteredValue() == -4);
	}
	
	////////////////////////////
	// Test that it still returns the median
	// after you insert a lot of values.
	////////////////////////////
	@Test
	public void testManyValuesA() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(4);
		
		f.add(1);
		f.add(12);
		f.add(8);
		f.add(4);
		f.add(-100);
		f.add(17);
		f.add(-35);
		f.add(3);
		f.add(2);
		
		assertTrue(f.getFilteredValue() == 11);
	}
	
	@Test
	public void testManyValuesB() {
		SmoothDifferenceFilter f = new SmoothDifferenceFilter(6);
		
		f.add(1);
		f.add(12);
		f.add(8);
		f.add(4);
		f.add(-100);
		f.add(17);
		f.add(-35);
		f.add(3);
		f.add(2);
		
		assertTrue(f.getFilteredValue() == 16);
	}
}
