package test;

import static org.junit.Assert.*;

import org.junit.Test;

import Master.Filter;


public class FilterTest {

	///////////////////////////////
	// Test case where we haven't added any samples.
	///////////////////////////////
	@Test
	public void testNoSamples() {
		Filter t = new Filter(3);
		
		assertTrue(t.getFilteredValue() == 0);
	}

	///////////////////////////////
	// Test base case
	///////////////////////////////
	@Test
	public void testAllZeros() {
		Filter t = new Filter(3);
		
		t.add(0);
		t.add(0);
		t.add(0);
		
		assertTrue(t.getFilteredValue() == 0);
	}
	
	///////////////////////////////
	// Test that only the last value is returned.
	///////////////////////////////
	@Test
	public void testLastValueA() {
		Filter t = new Filter(3);
		
		t.add(1);
		
		assertTrue(t.getFilteredValue() == 1);
	}

	@Test
	public void testLastValueB() {
		Filter t = new Filter(3);
		
		t.add(1);
		t.add(2);
		
		assertTrue(t.getFilteredValue() == 2);
	}

	@Test
	public void testLastValueC() {
		Filter t = new Filter(3);
		
		t.add(1);
		t.add(2);
		t.add(3);
		
		assertTrue(t.getFilteredValue() == 3);
	}

	@Test
	public void testLastValueD() {
		Filter t = new Filter(3);
		
		t.add(1);
		t.add(2);
		t.add(0);
		
		assertTrue(t.getFilteredValue() == 0);
	}

	///////////////////////////////
	// Test with larger size.
	///////////////////////////////
	@Test
	public void testLargeSize() {
		Filter t = new Filter(10);
		
		t.add(1);
		t.add(2);
		t.add(4);
		
		assertTrue(t.getFilteredValue() == 4);
	}

	///////////////////////////////
	// Test that calling getFilteredValue multiple times does not
	// break anything.
	///////////////////////////////
	@Test
	public void testMultipleQueries() {
		Filter t = new Filter(10);
		
		t.add(1);
		t.add(2);
		t.add(4);
		
		t.getFilteredValue();
		t.getFilteredValue();
		
		assertTrue(t.getFilteredValue() == 4);
	}
}
