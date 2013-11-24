package Master;

public class MedianFilter extends Filter {
	/**
	 * Constructor.
	 * 
	 */
	public MedianFilter(){
		super();
	}
	
	/**
	 * Constructor
	 * 
	 * @param windowSize Number of samples in window.
	 */
	public MedianFilter(int windowSize){
		super(windowSize);
	}
	
	@Override
	protected int filter(){
		return getMedian(raw);
	}
	
	/**
	 * Gets the median of an array.
	 * 
	 * @param values The array of values.
	 * 
	 * @return The median.
	 */
	private int getMedian(int values[]){
		int length = values.length;
		int sorted[] = new int[length];
		
		// Copy all values
		for(int i = 0; i < length; i++){
			sorted[i] = values[i];
		}
		
		sort(sorted);
		for(int i = 0; i < sorted.length; i++){
		}
		
		if(length % 2 == 0){
			return (sorted[length / 2] + sorted[length + 1]) / 2;
		}
		else{
			return sorted[length / 2];
		}
	}

	/**
	 * Sorts an array in-place.
	 * <p>
	 * Currently, does not use the best sorting algorithm, but it shouldn't
	 * matter for small arrays.
	 * 
	 * @param values The array to be sorted.
	 */
	private void sort(int values[]){
		for(int i = 0; i < values.length - 1; i++){
			for(int j = 0; j < values.length - 1 - i; j++){
				// If the next value is smaller, then switch these two.
				if(values[j] > values[j + 1]){
					int temp = values[j + 1];
					values[j + 1] = values[j];
					values[j] = temp;
				}
			}
		}
	}
}
