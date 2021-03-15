import java.util.ArrayList;

public class SimpleICP {
	
	ArrayList<double[]> featureMatches=null;//prevIndex,prevX,prevZ,,currIndex,currX,currZ,distance
	static double PROXIMITY_THRESHOLD = 5000;
	static double PHYSICAL_DISTANCE_THRESHOLD = 1;
	
	Transform prev = null;
	Transform curr = null;
	
	int prevMarked = -1;
	int currMarked = -1;
	

	public double[] computeShift1() {
		
		
		//first method will be applying translation so that the best matching points are at the same spot
		//and then rotating the entire cloud around that spot to see which orientation makes for the best
		//match between layers
		int bestScore = -1;
		double[] bestShift = new double[3];


		
		for(int x=0;x<20;x++) {
			for(int z=0;z<20;z++) {
				curr.translate((x-10)/5d, (z-10)/5d);
				curr.rotate((x-10)/5d, (z-10)/5d,-30);
				for(int r=-30;r<30;r++) {
					curr.rotate((x-10)/5d, (z-10)/5d, 1);
					int score = curr.computeMatchingScore(prev);
					if(bestScore<score) {
						bestShift= new double[]{curr.points[0][0],curr.points[0][2],r};
						bestScore=score;
					}
				}
				curr.resetPositions();
				
			}
		}
		
		
		
		
		
		//return new double[] {0,0,0};
		return new double[] {bestShift[0],bestShift[1],bestShift[2]};
	}
	
	public SimpleICP(FeatureCloud t_minus_1, FeatureCloud t) {
		
		prev = new Transform(t_minus_1);
		curr = new Transform(t);
		//now we're matching the features
		this.featureMatches = new ArrayList<double[]>();
		match_features(t_minus_1,t);
		
		System.out.println(featureMatches.size());
		
		//itteratively apply transform that will bring both matched features closer together
		
		
	}
	
	public void match_features(FeatureCloud Left, FeatureCloud Right) {
		
		for(int fl=0;fl<Left.features.size();fl++) {
		double[] vectorL = Left.features.get(fl);
		double Lx = Left.featurePositions.get(fl)[0];
		double Ly = Left.featurePositions.get(fl)[1];
		double Rx = 0;
		double Ry =0;
		
		double smallest =999999999;
		int smallest_index=1;
		
		for(int fr=0;fr<Right.features.size();fr++) {
			
			double[] vectorR = Right.features.get(fr);
			Rx = Right.featurePositions.get(fr)[0];
			Ry = Right.featurePositions.get(fr)[1];
			
			
			if(Rx<Lx&&Math.abs(Ly-Ry)<3) {
				double distance =0;
				for(int v =0;v<128;v++) {
					distance+=Math.abs(vectorR[v]-vectorL[v]);
				}
				
				if(distance<smallest) {
					smallest=distance;
					smallest_index=fr;
				}	
			}	
		}
		
		if(smallest<PROXIMITY_THRESHOLD) {
			double dx = Left.featurePositions.get(fl)[0]-Right.featurePositions.get(smallest_index)[0];
			double dz = Left.featurePositions.get(fl)[2]-Right.featurePositions.get(smallest_index)[2];
			double trueDistance = Math.sqrt(dx*dx+dz*dz);
			if(trueDistance<PHYSICAL_DISTANCE_THRESHOLD)
			featureMatches.add(new double[] {
					fl,Left.featurePositions.get(fl)[0],Left.featurePositions.get(fl)[2],
					smallest_index,Right.featurePositions.get(smallest_index)[0],Right.featurePositions.get(smallest_index)[2],
					smallest});
			
			
		}
		
		
	}
		
		
		
		
	}
}

class Transform{
	double[][] points;
	final double[][] pointsFinal;
	double MINIMAL_DISTANCE = 0.5d;
	
	//computes mass center of points within radius
	public double[] computeCenter(double radius) {
		
		double xc =0;
		double zc =0;
		double c =0;
		
		for(int i=0;i<this.points.length;i++) {
			double x = this.points[i][0];
			double z = this.points[i][2];
			//distance from 0,0
			double distance = Math.sqrt((x*x)+(z*z));
			if(distance<=radius) {
				c++;
				xc+=x;
				zc+=z;
			}
			
		}
				
		
		return new double[] {xc/c,zc/c}; 
	}
	
	public void resetPositions() {
		
		for(int i=0;i<points.length;i++) {
			points[i]=pointsFinal[i];
		}
		
	}
	
	public int computeMatchingScore(Transform another) {
		
		//the comparing strategy is this: we're counting points from group B (prev) that are in close proximity
		// to any of the points in group A (curr)
		int count=0;
		
		for(int i=0;i<another.points.length;i++) {
			
			boolean hasMatch = false;
			double anotherX = another.points[i][0];
			double anotherZ = another.points[i][2];
			
			for(int u=0;u<this.points.length;u++) {
				double xShiftSquared = Math.pow(this.points[u][0]-anotherX,2);
				double zShiftSquared = Math.pow(this.points[u][2]-anotherZ,2);
				double distance = Math.sqrt(xShiftSquared+zShiftSquared);
				
				
				if(distance<=MINIMAL_DISTANCE) {hasMatch=true; u=this.points.length;}
			}
			if(hasMatch)count++;
			
		}
		
		return count;
	}
	
	public void translate(double x, double z) {
		

		for(int i =0;i<points.length;i++) {
			double currX = points[i][0];
			double currY = points[i][1];
			double currZ = points[i][2];
			
			points[i]=new double[] {currX+x,currY,currZ+z};
			
		}
		
	}
	
	public void rotate(double pivotX, double pivotZ, double rotation){
		
	
		
		for(int i =0;i<points.length;i++) {

			double xShift = (points[i][0]-pivotX);
			double yShift = (points[i][2]-pivotZ);
			double angle = Math.toDegrees(Math.atan(yShift==0?(xShift*99999999):(xShift/yShift)));
		    
		    double result=0;
		    if(xShift<=0) {
		    	if(yShift<0) {result=90-angle;}
		    	else {result=270-angle;}}
		    else {
		    	if(yShift<0) {result=90-angle;}
		    	else {result=270-angle;}}
		    
		    double radians = ((-rotation+result)/360d)*Math.PI*2d;
		    double distance = Math.sqrt(Math.pow(xShift, 2)+Math.pow(yShift, 2));
		    double cz = pivotZ-Math.sin(radians)*distance;
		    double cx = pivotX-Math.cos(radians)*distance;
		    double cy = points[i][1]; 
			
			points[i]=new double[] {cx,cy,cz};
			
		}
		
		
	} 
	
	public Transform(FeatureCloud FC) {
		pointsFinal = new double[FC.featurePositions.size()+1][3];
		points = new double[FC.featurePositions.size()+1][3];
		points[0]=new double[] {0d,0d,0d};
		pointsFinal[0]=new double[] {0d,0d,0d};
		for(int i=1;i<FC.featurePositions.size()+1;i++) {
			points[i]=FC.featurePositions.get(i-1);
			pointsFinal[i]=FC.featurePositions.get(i-1);
		}
	}
	
	public Transform(double[][] FC) {
		pointsFinal = new double[FC.length+1][3];
		points = new double[FC.length+1][3];
		points[0]=new double[] {0d,0d,0d};
		pointsFinal[0]=new double[] {0d,0d,0d};
		for(int i=1;i<FC.length+1;i++) {
			points[i]=FC[i-1];
			pointsFinal[i]=FC[i-1];
		}
	}
}