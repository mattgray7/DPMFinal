package Master;

public class PathGenerator {

	private Odometer odometer;
	
	private double BORDER_DIST = 10.0;		//border distance for checking if a point is in an area
	private double GREEN_BORDER_DIST = 15.0;	//must be larger than BORDER_DIST, for travelling to border point
	
	private double gx0=60;			//green zone left x component
	private double gx1=90;			//green zone right x component
	private double gy0=40;			//green zone lower y component
	private double gy1=90;			//green zone upper y component
	
	private double rx0=600;			//red zone left x component
	private double rx1=900;			//red zone right x component
	private double ry0=600;			//red zone lower y component
	private double ry1=900;			//red zone upper y component
	
	private double wx0 = -30.0;			//left wall
	private double wx1 = 330.0;			//right wall
	private double wy0 = -30.0;			//lower wall
	private double wy1 = 330.0;			//upper wall
	

	
	public PathGenerator(Odometer odo) {
		this.odometer = odo;
	}
	
	public double[] calculateBorderPoint(){
		double x = odometer.getX();
		double y = odometer.getY();
		double[] point = new double[2];
		if(x <= gx0 && y <= gy0){
			point[0] = gx0 - GREEN_BORDER_DIST;
			point[1] = gy0 - GREEN_BORDER_DIST;
		}else if (x <= gx0 && y >= gy0){
			point[0] = gx0 - GREEN_BORDER_DIST;
			point[1] = gy1 + GREEN_BORDER_DIST;
		}else if (x >= gx0 && y <= gy0){
			point[0] = gx1 + GREEN_BORDER_DIST;
			point[1] = gy0 - GREEN_BORDER_DIST;
		}else{
			point[0] = gx1 + GREEN_BORDER_DIST;
			point[1] = gy1 + GREEN_BORDER_DIST;
		}
		return point;
	}
	public double[][] findClosestCorner() {
		double[] corner = new double [2];
		double [][] corners = new double [4][2];
		if(Math.abs(rx1 - odometer.getX()) < Math.abs(rx0 - odometer.getX())) {
			corner[0] = rx1;
		}
		else {
			corner[0] = rx0;
		}
		
		if(Math.abs(ry1 - odometer.getY()) < Math.abs(ry0 - odometer.getY())) {
			corner[1] = ry1;
		}
		else {
			corner[1] = ry0;
		}
		//Check which corner is closest and then assign route for visiting each corner
		if (corner[0] == rx0 && corner[1] == ry0) {
			corners[0][0] = rx0;
			corners[0][1] = ry0;
			corners[1][0] = rx1;
			corners[1][1] = ry0;
			corners[2][0] = rx1; 
			corners[2][1] = ry1;
			corners[3][0] = rx0;
			corners[3][1] = ry1;
		}
		else if (corner[0] == rx1 && corner[1] == ry0) {
			corners[0][0] = rx1;
			corners[0][1] = ry0;
			corners[1][0] = rx1;
			corners[1][1] = ry1;
			corners[2][0] = rx0; 
			corners[2][1] = ry1;
			corners[3][0] = rx0;
			corners[3][1] = ry0;
		}
		else if (corner[0] == rx1 && corner[1] == ry1) {
			corners[0][0] = rx1;
			corners[0][1] = ry1;
			corners[1][0] = rx0;
			corners[1][1] = ry1;
			corners[2][0] = rx0; 
			corners[2][1] = ry0;
			corners[3][0] = rx1;
			corners[3][1] = ry0;
		} 
		else {
			corners[0][0] = rx0;
			corners[0][1] = ry1;
			corners[1][0] = rx0;
			corners[1][1] = ry0;
			corners[2][0] = rx1; 
			corners[2][1] = ry0;
			corners[3][0] = rx1;
			corners[3][1] = ry1;
		}
		return corners;
	} 
	public double[] generateSquarePath(double x, double y, double depositAngle){
		double[] path = new double[4];	//[x1, y1, x2, y2]
		double borderX;
		double borderY;

		if(checkPointsInPath(odometer.getX(), odometer.getY(), odometer.getX(), y) 
				&& (checkPointsInPath(odometer.getX(), y, x, y))){
			path[0] = odometer.getX();
			path[1] = y;
			path[2] = x;
			path[3] = y;
		}else if (checkPointsInPath(odometer.getX(), odometer.getY(), x, odometer.getY()) 
				&& (checkPointsInPath(x, odometer.getY(), x, y))){
			path[0] = x;
			path[1] = odometer.getY();
			path[2] = x;
			path[3] = y;
		}
		
		return path;

	}
	
	public Boolean checkPointAhead(double angle, int distance){
		double nx = 0;
		double ny = 0;
		nx = distance*Math.cos(angle*Math.PI/180) + odometer.getX();
		ny = distance*Math.sin(angle*Math.PI/180) + odometer.getY();
		return checkPoint(nx, ny, BORDER_DIST);
	}
	
	
	
	public boolean checkPointsInPath(double x0, double y0, double x1, double y1) {
		
		double currentX = odometer.getX();
		double startX = currentX;
		double currentY = odometer.getY();
		double startY = currentY;
		double heading = Math.atan2(y1 - y0, x1 - x0) * (180.0/ Math.PI);

		//Check if any point along path intersects the red zone
		   
		if((x1 - currentX) > 0){
			while(currentX <= x1){
		       currentY = (currentX - startX) * Math.tan(heading * Math.PI/180) + startY;
		       if(!checkPoint(currentX, currentY, BORDER_DIST)){
		    	   return false;
		       }else{
		    	   currentX++;
		       }
			}
		}else if ((x1 - currentX) < 0){
		     while(currentX >= x1){
		       currentY = (startX - currentX) * Math.tan((180.0 - heading) * Math.PI/180) + startY;
		       if (!checkPoint(currentX, currentY, BORDER_DIST)){
		    	   return false;
		       }else{
		        currentX--;
		       }
		     }
		     
		   }else if (Math.abs(x1 - currentX) < 1){
			   if(y1 > odometer.getY()){
			     while(currentY <= y1){
				       if (!checkPoint(currentX, currentY, BORDER_DIST)){
				    	   return false;
				       }else{
				        currentY++;
				       }
			     }
			   }else{
				     while(currentY >= y1){
					       if (!checkPoint(currentX, currentY, BORDER_DIST)){
					    	   return false;
					       }else{
					        currentY--;
					       }
				     }
			   }
		   }
		   return true;
	}

	
	
	/**
	 * Will generate a path of points that lead from current location
	 * to the green zone.
	 * @return void
	 */
	//dont think this method will be used
	public void generatePathToGreen(){
		//method will need to be changed to handle walls and green zone, not going to comment until algorithm is complete
	    double x = odometer.getX();
	    double y = odometer.getY();
	    
	    double x0 = gx0;
	    double y0 = gy0;
	    double x1 = gx1;
	    double y1 = gy1;
	    
	    double xDest = (x0 + x1)/2.0;
	    double yDest = (y0 + y1)/2.0;
	    
	    double[] xpath = new double[40];
	    double[] ypath = new double[40];
	    double length = xpath.length;
	    
	    //should include x+3-, y+30, but currently does not
	    
	    xpath[0] = x;     //first point manually chosen
	    ypath[0] = y;

	    
	    for(int i=0; i< xpath.length - 1; i++){
	        
	        //left and below green zone
	        
	        if(xpath[i] <= x0){
	          if ((xDest - xpath[i]) > 30.0){
	            xpath[i+1] = xpath[i] + 30.0;
	            ypath[i+1] = ypath[i];
	          }else{
	            xpath[i+1] = xDest;
	            ypath[i+1] = ypath[i];
	            
	            if(ypath[i+1] == yDest && xpath[i+1] == xDest){
	            	  length = i+1;
	            	  break;
	            }
	            
	          }
	        }else if (xpath[i] >= x1){
	          if ((xpath[i] - xDest) > 30.0){
	            xpath[i+1] = xpath[i] - 30.0;
	            ypath[i+1] = ypath[i];
	          }else{
	            xpath[i+1] = xDest;
	            ypath[i+1] = ypath[i];
	            
	            if(ypath[i+1] == yDest && xpath[i+1] == xDest){
	            	  length = i+1;
	            	  break;
	            }
	            
	          }
	        }else if (ypath[i] <= y0){
	          if ((yDest - ypath[i]) > 30.0){
	            ypath[i+1] = ypath[i] + 30.0;
	            xpath[i+1] = xpath[i];
	          }else{
	            ypath[i+1] = yDest;
	            xpath[i+1] = xpath[i];
	            
	            if(ypath[i+1] == yDest && xpath[i+1] == xDest){
	            	  length = i+1;
	            	  break;
	            }
	            
	          }
	        }else{
	          if ((ypath[i] - yDest) >= 30.0){
	            ypath[i+1] = ypath[i] - 30.0;
	            xpath[i+1] = xpath[i];
	          }else{
	            ypath[i+1] = yDest;
	            xpath[i+1] = xpath[i];
	            
	            if(ypath[i+1] == yDest && xpath[i+1] == xDest){
	            	  length = i+1;
	            	  break;
	            }
	            
	          }
	        }
	      }
	    
	    /*
	    xDestination = xDest;
	    yDestination = yDest;
	    
	    for(int i=0; i < length; i++){
	    	xPath[i] = xpath[i];
	    	yPath[i] = ypath[i];
	    }*/
	 }

	public boolean checkPoint(double x, double y, double border) {
	    //check if next point is within a wall
	    if((x <= wx0 + border) || (x >= wx1 - border) || (y <= wy0 + border) || (y >= wy1 - border)){
	      return false;
	    }
	    
	    //check if next point is within red zone threshold
	    if ((x >= rx0 - border) && (x <= rx1 + border) && (y >= ry0 - border) && (y <= ry1 + border)){
	      return false;
	    }
	    

	    	//check if next point is within green zone threshold
	    	if ((x >= gx0 - border) && (x <= gx1 + border) && (y >= gy0 - border) && ( y <= gy1 + border)){
	    		return false;
	    	}
	    
	    
	    return true;
	}


	
}
