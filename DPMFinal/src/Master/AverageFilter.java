package Master;

/**
 * Type of filter that returns the average value of the last few raw values.
 * 
 * @author Nicholas Aird
 */
public class AverageFilter extends Filter {
	/**
	 * Constructor.
	 * 
	 */
	public AverageFilter(){
		super();
	}
	
	/**
	 * Constructor
	 * 
	 * @param windowSize Number of samples in window.
	 */
	public AverageFilter(int windowSize){
		super(windowSize);
	}
	
	@Override
	protected int filter(){
		return average(raw);
	}
	
	/**
	 * Calculates the average of all the values in the array.
	 * 
	 * @param array
	 * 
	 * @return The average value.
	 */
	private int average(int array[]){
		int total = 0;
		
		for(int i = 0; i < array.length; i++){
			total += array[i];
		}
		
		return total / array.length;
	}
}
