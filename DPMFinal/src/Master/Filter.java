package Master;

/**
 * Container that can filter values.
 * <p>
 * The behavior for getFilteredValue() is:
 * - When no samples have been added, it returns 0.
 * - When not enough samples have yet been added and the window is not filled,
 *   it returns the last unfiltered value.
 * - When the group of samples fill the window, it returns the normal filtered
 *   value.
 * 
 * @author Nick
 *
 */
public class Filter {
	protected int windowSize;
	protected int raw[];
	protected int filtered[];
	
	protected int numberOfSamples;
	
	/**
	 * Default constructor.
	 * 
	 */
	public Filter(){
		windowSize = 4;
		raw = new int[windowSize];
		filtered = new int[windowSize];
		
		numberOfSamples = 0;
	}
	
	/**
	 * Constructor.
	 * 
	 * @param sizeOfWindow Number of previous samples used to calculate the filtered value.
	 */
	public Filter(int sizeOfWindow){
		this.windowSize = sizeOfWindow;
		raw = new int[sizeOfWindow];
		filtered = new int[sizeOfWindow];
		
		numberOfSamples = 0;
	}
	
	/**
	 * Get the current filtered value.
	 * 
	 * @return The current filtered value.
	 */
	public int getFilteredValue(){
		if(numberOfSamples == 0){
			return 0;
		}
		else return filtered[windowSize - 1];
	}
	
	/**
	 * Add a new sample.
	 * <p>
	 * This will erase the oldest sample if the group of samples is already
	 * filled to its capacity.
	 * 
	 * @param sample New value to insert.
	 */
	public void add(int sample){
		translateArrays();

		incrementNumberOfSamples();
		
		raw[windowSize - 1] = sample;
		filtered[windowSize - 1] = calculateFilteredValue();
	}
	
	/**
	 * Shift all value to the left by one.
	 * <p>
	 * This leaves the last value untouched.
	 * 
	 */
	private void translateArrays(){
		for(int i = 0; i < windowSize - 1; i++){
			raw[i] = raw[i + 1];
			filtered[i] = filtered[i + 1];
		}
	}
	
	/**
	 * Returns the filtered value using the current window of samples.
	 * <p>
	 * If not enough samples were given, then it returns the raw value.
	 * 
	 * @return The filtered value for the current window of samples.
	 */
	private int calculateFilteredValue(){
		if(numberOfSamples == 0){
			return 0;
		}
		else if(numberOfSamples < windowSize){
			return raw[windowSize - 1];
		}
		else{
			return filter();
		}
	}
	
	/**
	 * Get the filtered value, assuming the raw array is full of valid values.
	 * @return
	 */
	protected int filter(){
		return raw[windowSize - 1];
	}
	
	/**
	 * Increment the number of samples contained in the group.
	 * <p>
	 * If you just started adding samples, then this number would grow.
	 * However, if you already have "windowSize" number of samples, then it
	 * will not increment.
	 */
	private void incrementNumberOfSamples(){
		if(numberOfSamples < windowSize){
			numberOfSamples++;
		}
	}
}
