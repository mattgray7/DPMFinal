package Master;


/**
 * Filter that smooths out the last few raw values and approximates the slope (derivative).
 * 
 * @author Nicholas Aird
 */
public class SmoothDifferenceFilter extends Filter {
	/**
	 * Constructor.
	 * 
	 */
	public SmoothDifferenceFilter(){
		super();
	}
	
	/**
	 * Constructor.
	 * 
	 * @param windowSize Number of samples in window.
	 */
	public SmoothDifferenceFilter(int windowSize){
		super(windowSize);
	}
	
	@Override
	protected int filter(){
		if(numberOfSamples == 0){
			return 0;
		}
		else if(numberOfSamples < windowSize){
			return raw[windowSize];
		}
		
		double total = 0.0;
		double weight = 1.0 / (windowSize / 2.0);
		
		// Apply weight -1/2 to all samples in the first half.
		for(int i = 0; i < windowSize / 2; i++){
			total += -weight * raw[i];
		}

		// Apply weight +1/2 to all samples in the last half.
		for(int i = (windowSize + 1) / 2; i < windowSize; i++){
			total += weight * raw[i];
		}
		
		return (int)total;
	}
}
