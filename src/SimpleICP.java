import java.util.ArrayList;

public class SimpleICP {
	
	ArrayList<double[]> featureMatches=null, fetureCopied=null;//prevIndex,prevX,prevZ,,currIndex,currX,currZ,distance
	static double PROXIMITY_THRESHOLD = 10000;
	static double PHYSICAL_DISTANCE_THRESHOLD = 2;
	Transform prev = null;
	Transform curr = null;
	int prevMarked = -1;
	int currMarked = -1;
	boolean useKalman=false;

	

	

	public double[] computeShift1() {
		
		//first method will be applying translation so that the best matching points are at the same spot
		//and then rotating the entire cloud around that spot to see which orientation makes for the best
		//match between layers
		int bestScore = -1;
		double[] bestShift = new double[3];


		
		for(int x=0;x<20;x++) {
			for(int z=0;z<20;z++) {
				curr.translate((x-10)/10d, (z-10)/10d);
				curr.rotate((x-10)/10d, (z-10)/10d,-30);
				for(int r=-30;r<30;r++) {
					curr.rotate((x-10)/10d, (z-10)/10d, 1);
					int score = curr.computeMatchingScore(prev);
					if(bestScore<score) {
						bestShift= new double[]{curr.points[0][0],curr.points[0][2],r};
						bestScore=score;
					}
				}
				curr.resetPositions();
				
			}
		}
		
		
		
		
		

		return new double[] {bestShift[0],bestShift[1],bestShift[2]};
	}

	public double[] RANSAC() {
		
		int bestScore=0;
		double[] best= new double[3];
		//looking for a, x, z values
		   //pick 3 best matches 
		for(int p=0;p<10;p++) {
			
		copyFeatures();
		double[][] matchedPoints = takeBestMAtches();
		double[] transform = computeTransform(matchedPoints);
		curr.rotate(0, 0, -transform[2]);
		curr.translate(transform[0],transform[1]);
		int score = curr.computeMatchingScore(prev);
		curr.resetPositions();
		
		if(bestScore<score) {
			bestScore=score;
			best=transform;
		}
		
		
		
	}
		double[] answer = best;
		return answer;
	}
	
	
	void copyFeatures() {
		fetureCopied = new ArrayList<double[]>();
		
		for(int i=0;i<featureMatches.size();i++) {
			double [] c = featureMatches.get(i);
			fetureCopied.add(new double[] {c[0],c[1],c[2],c[3],c[4],c[5],c[6],});
		}
	}
	
	double[] computeTransform(double[][] m) {
		
		Point Ap = new Point(m[0][1],m[0][2]);
		Point A = new Point(m[0][4],m[0][5]);
		
		Point Bp = new Point(m[1][1],m[1][2]);
		Point B = new Point(m[1][4],m[1][5]);
		
		Point Cp = new Point(m[2][1],m[2][2]);
		Point C = new Point(m[2][4],m[2][5]);
		//constructing equation 1 from match 1 and 3
		double equals1 = Cp.y-Ap.y;
		double sin1 = -C.y-(-A.y);
		double cos1 = C.x-A.x;
		//constructing equation 1 from match 1 and 3
		double equals2 = Cp.y-Bp.y;
		double sin2 = -C.y-(-B.y);
		double cos2 = C.x-B.x;

		double[] histogram1 = solveTrygonometricEquation(equals1,sin1,cos1);
		double[] histogram2 = solveTrygonometricEquation(equals2,sin2,cos2);
		
		double smallest=10;
		double smallestIndex=-300;
		for(int i=-100;i<101;i++) {
			double value = histogram1[i+100]+histogram2[i+100];
		if(value<smallest) {
			smallestIndex=i;
			smallest=value;
		}
			
		}
		double rotation = (Math.PI*(double)smallestIndex)/100d;
		
		double h1=Ap.x-(A.x*Math.cos(rotation)-A.y*Math.sin(rotation));
		double h2=Bp.x-(B.x*Math.cos(rotation)-B.y*Math.sin(rotation));
		double h3=Cp.x-(C.x*Math.cos(rotation)-C.y*Math.sin(rotation));
		
		double k1=Ap.y-(A.x*Math.sin(rotation)+A.y*Math.cos(rotation));
		double k2=Bp.y-(B.x*Math.sin(rotation)+B.y*Math.cos(rotation));
		double k3=Cp.y-(C.x*Math.sin(rotation)+C.y*Math.cos(rotation));
		
		return new double[]{h1,k1,smallestIndex};
	}
	
	double[] solveTrygonometricEquation(double equals, double sin, double cos) {
		
		double[] histogram = new double[201];
		
		for(int i=-100;i<101;i++) {
			double angle = (Math.PI*(double)i)/100d;
			double value = sin*Math.sin(angle)+cos*Math.cos(angle);
			double distance = Math.abs(value-equals);
			histogram[i+100]=distance;
		}
		
		return histogram;
	}
	
	
	
	double[][] takeBestMAtches(){
		double[][] pickedMatches = new double[3][7];
		
		double first=0,second=0,third=0;
		int firstIndex=0,secondIndex=0,thirdIndex=0;
		
		for(int i=0;i<fetureCopied.size();i++) {
			double d = fetureCopied.get(i)[6];
			
			if(d>first) {firstIndex=i;first=d;}
			else if(d>second) {secondIndex=i;second=d;}
			else if(d>third) {thirdIndex=i;third=d;}
			
		}
		
		pickedMatches[0]=fetureCopied.get(firstIndex);
		pickedMatches[1]=fetureCopied.get(secondIndex);
		pickedMatches[2]=fetureCopied.get(thirdIndex);
		
		featureMatches.remove(firstIndex);
		
		return pickedMatches;
	}
	
	
	
	private double InferRotation(Point A, Point B) {
		//Point A acts as a pivot
		double yShift = (B.x-A.x);
		double xShift = (B.y-A.y);
		double angle = Math.toDegrees(Math.atan(yShift==0?(xShift*99999999):(xShift/yShift)));
	    
	    double result=0;
	    if(xShift<=0) {
	    	if(yShift<0) {result=90-angle;}
	    	else {result=270-angle;}}
	    else {
	    	if(yShift<0) {result=90-angle;}
	    	else {result=270-angle;}}
		
		
		
		return result;
	}
	
	public SimpleICP(FeatureCloud t_minus_1, FeatureCloud t, boolean kalman) {
		
		useKalman=kalman;
		prev = new Transform(t_minus_1);
		curr = new Transform(t);
		//now we're matching the features
		this.featureMatches = new ArrayList<double[]>();
		match_features(t_minus_1,t);
		
		//System.out.println(featureMatches.size());
		
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
	double MINIMAL_DISTANCE = 0.1d;
	
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