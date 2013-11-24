package test;

import static org.junit.Assert.*;

import org.junit.Test;

import Master.AverageFilter;


public class AverageFilterTest {

	////////////////////////////
	// Test that the behavior from Filter is conserved.
	////////////////////////////
	@Test
	public void testFilterA() {
		AverageFilter f = new AverageFilter(3);
		
		assertTrue(f.getFilteredValue() == 0);
	}

	@Test
	public void testFilterB() {
		AverageFilter f = new AverageFilter(3);
		
		f.add(1);
		
		assertTrue(f.getFilteredValue() == 1);
	}

	@Test
	public void testFilterC() {
		AverageFilter f = new AverageFilter(3);
		
		f.add(1);
		f.add(2);
		
		assertTrue(f.getFilteredValue() == 2);
	}

	/////////////////////////////
	// Test that it returns the average.
	/////////////////////////////
	@Test
	public void testAverageA() {
		AverageFilter f = new AverageFilter(3);
		
		f.add(1);
		f.add(12);
		f.add(3);
		
		assertTrue(f.getFilteredValue() == ((1 + 12 + 3) / 3));
	}

	@Test
	public void testAverageB() {
		AverageFilter f = new AverageFilter(3);
		
		f.add(1);
		f.add(12);
		f.add(100);
		
		assertTrue(f.getFilteredValue() == ((1 + 12 + 100) / 3));
	}

	@Test
	public void testAverageC() {
		AverageFilter f = new AverageFilter(3);
		
		f.add(5);
		f.add(12);
		f.add(3);
		
		assertTrue(f.getFilteredValue() == ((5 + 12 + 3) / 3));
	}

	@Test
	public void testAverageD() {
		AverageFilter f = new AverageFilter(3);
		
		f.add(1);
		f.add(-1);
		f.add(-4);
		
		assertTrue(f.getFilteredValue() == ((1 - 1 - 4) / 3));
	}
	
	////////////////////////////
	// Test that it still returns the average
	// after you insert a lot of values.
	////////////////////////////
	@Test
	public void testManyValues() {
		AverageFilter f = new AverageFilter(3);
		
		f.add(1);
		f.add(12);
		f.add(8);
		f.add(4);
		f.add(-100);
		f.add(17);
		f.add(-35);
		f.add(3);
		f.add(2);
		
		assertTrue(f.getFilteredValue() == ((-35 + 3 + 2) / 3));
	}
}
