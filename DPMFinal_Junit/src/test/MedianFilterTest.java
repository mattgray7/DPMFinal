package test;

import static org.junit.Assert.*;

import org.junit.Test;

import Master.MedianFilter;


public class MedianFilterTest {

	////////////////////////////
	// Test that the behavior from Filter is conserved.
	////////////////////////////
	@Test
	public void testFilterA() {
		MedianFilter f = new MedianFilter(3);
		
		assertTrue(f.getFilteredValue() == 0);
	}

	@Test
	public void testFilterB() {
		MedianFilter f = new MedianFilter(3);
		
		f.add(1);
		
		assertTrue(f.getFilteredValue() == 1);
	}

	@Test
	public void testFilterC() {
		MedianFilter f = new MedianFilter(3);
		
		f.add(1);
		f.add(2);
		
		assertTrue(f.getFilteredValue() == 2);
	}

	/////////////////////////////
	// Test that it returns the median.
	/////////////////////////////
	@Test
	public void testMedianA() {
		MedianFilter f = new MedianFilter(3);
		
		f.add(1);
		f.add(12);
		f.add(3);
		
		assertTrue(f.getFilteredValue() == 3);
	}

	@Test
	public void testMedianB() {
		MedianFilter f = new MedianFilter(3);
		
		f.add(1);
		f.add(12);
		f.add(100);
		
		assertTrue(f.getFilteredValue() == 12);
	}

	@Test
	public void testMedianC() {
		MedianFilter f = new MedianFilter(3);
		
		f.add(5);
		f.add(12);
		f.add(3);
		
		assertTrue(f.getFilteredValue() == 5);
	}

	@Test
	public void testMedianD() {
		MedianFilter f = new MedianFilter(3);
		
		f.add(1);
		f.add(-1);
		f.add(-4);
		
		assertTrue(f.getFilteredValue() == -1);
	}
	
	////////////////////////////
	// Test that it still returns the median
	// after you insert a lot of values.
	////////////////////////////
	@Test
	public void testManyValues() {
		MedianFilter f = new MedianFilter(3);
		
		f.add(1);
		f.add(12);
		f.add(8);
		f.add(4);
		f.add(-100);
		f.add(17);
		f.add(-35);
		f.add(3);
		f.add(2);
		
		assertTrue(f.getFilteredValue() == 2);
	}
}
