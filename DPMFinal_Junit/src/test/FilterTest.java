package test;

import static org.junit.Assert.*;

import org.junit.Test;

import Master.Filter;


public class FilterTest {

	///////////////////////////////
	// Test case where we haven't added any samples.
	///////////////////////////////
	@Test
	public void testNoSamplesFiltered() {
		Filter f = new Filter(3);
		
		assertTrue(f.getFilteredValue() == 0);
	}

	@Test
	public void testNoSamplesRaw(){
		Filter f = new Filter(3);
		
		assertTrue(f.getRawValue() == 0);
	}
	
	///////////////////////////////
	// Test that getRawValue() returns the last raw value
	///////////////////////////////
	@Test
	public void testRawValueB(){
		Filter f = new Filter(3);
		
		f.add(1);
		
		assertTrue(f.getRawValue() == 1);
	}

	@Test
	public void testRawValueC(){
		Filter f = new Filter(3);
		
		f.add(1);
		f.add(2);
		
		assertTrue(f.getRawValue() == 2);
	}

	@Test
	public void testRawValueD(){
		Filter f = new Filter(3);

		f.add(1);
		f.add(2);
		f.add(3);
		
		assertTrue(f.getRawValue() == 3);
	}

	@Test
	public void testRawValueE(){
		Filter f = new Filter(3);

		f.add(1);
		f.add(2);
		f.add(3);
		f.add(4);
		
		assertTrue(f.getRawValue() == 4);
	}
	
	///////////////////////////////
	// Test that only the last filtered value is returned.
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
	public void testLargeSizeA() {
		Filter t = new Filter(10);
		
		t.add(1);
		t.add(2);
		t.add(4);
		
		assertTrue(t.getFilteredValue() == 4);
	}
	
	@Test
	public void testLargeSizeB() {
		Filter t = new Filter(10);
		
		t.add(1);
		t.add(2);
		t.add(4);
		
		assertTrue(t.getRawValue() == 4);
	}

	///////////////////////////////
	// Test that calling getFilteredValue multiple times does not
	// break anything.
	///////////////////////////////
	@Test
	public void testMultipleQueriesFiltered() {
		Filter t = new Filter(10);
		
		t.add(1);
		t.add(2);
		t.add(4);
		
		t.getFilteredValue();
		t.getFilteredValue();
		
		assertTrue(t.getFilteredValue() == 4);
	}

	///////////////////////////////
	// Test that calling getRawValue multiple times does not
	// break anything.
	///////////////////////////////
	@Test
	public void testMultipleQueriesRaw() {
		Filter t = new Filter(10);
		
		t.add(1);
		t.add(2);
		t.add(4);
		
		t.getRawValue();
		t.getRawValue();
		
		assertTrue(t.getRawValue() == 4);
	}
}
