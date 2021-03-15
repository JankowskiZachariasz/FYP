import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Simulator {
	
	double[][] map;
	Transform mapTransform = null;
	boolean absolutePositions = false;//true = absolutePositions, false = relative to the starting points
	boolean camView = false; //if true map doesn't require any transformations
	boolean photogrammetryPoints = false;
	ANNdata ann=null;
	ArrayList<ANNdata> trajectories = null;
	double[][] vertices = null;
	
	//loop-related variables
	int t =0;
	int s =0;
	
	public Simulator(boolean absolute,boolean camView, boolean photogrammetryPoints) {
		this.absolutePositions = absolute;
		this.camView = camView;
		this.photogrammetryPoints = photogrammetryPoints;
	}
	
	public void simulate(){
		
		for(int t=0;t<trajectories.size();t++) {
			
			ANNdata current = trajectories.get(t);
			this.ann = current;
//			Main.ann = this.ann;
			//let's determine number of steps
			int steps = current.shifts.size();
			
			//now we either leave the positions as they are (absolute by default) or we set it to 0s
			
			double x = current.positionX;
			double z = current.positionY;
			double r = current.rotation;
			
			if(!absolutePositions)
			{
				current.positionX=0d;
				current.positionY=0d;
				current.rotation=0d;
			}
			
			for(int s=0;s<steps;s++) {
				
				mapTransform.resetPositions();
				//mapTransform.
				double[] change = current.shifts.get(s);
				x+=change[0];
				z+=change[1];
				r+=change[2];
				mapTransform.translate(z, -x);
				mapTransform.rotate(0, 0, -r);
				
				
				current.generateAnnStep(mapTransform,s+1,photogrammetryPoints);
				
				try {TimeUnit.SECONDS.sleep(1);}catch(Exception exc) {}
				
				
			}
			
		}
		
	}
	
	public void simulateOneStep(){
	
		if(this.t<trajectories.size()){
			
			ANNdata current = trajectories.get(t);
			this.ann = current;
//			Main.ann = this.ann;
			//let's determine number of steps
			int steps = current.shifts.size();
			
			//now we either leave the positions as they are (absolute by default) or we set it to 0s
			
			double x = current.positionX;
			double z = current.positionY;
			double r = current.rotation;
			
			if(!absolutePositions)
			{
				current.positionX=0d;
				current.positionY=0d;
				current.rotation=0d;
			}
			
			if(this.s<steps) {
				
				mapTransform.resetPositions();
				//mapTransform.
				double[] change = current.shifts.get(s);
				x+=change[0];
				z+=change[1];
				r+=change[2];
				if(!camView) {
					mapTransform.translate(z, -x);
					mapTransform.rotate(0, 0, -r);
				}
				
				
				
				current.generateAnnStep(mapTransform,s+1,photogrammetryPoints);
				
				//try {TimeUnit.SECONDS.sleep(1);}catch(Exception exc) {}
				
				this.s++;
			}
			else {this.t++; this.s=0;}
			
		}
		
	}
	
	public void setMapVerticies(double[][] vertices) {
		//vertices -> Line segments -> points
		
		ArrayList<double[]> points = new ArrayList<double[]>();
		
		//number of vertices in a polygon = number of sides in that polygon
		for(int i = 0; i<vertices.length;i++) {//i determines point A
			int iB =0;
			if(i<vertices.length-1) iB=i+1;
			
			double[] A = vertices[i];
			double[] B = vertices[iB];
			
			
			Segment segment = new Segment(A[0],A[1],B[0],B[1]);
			ArrayList<double[]> segmentPoints = segment.points();
			for(int sp=0;sp<segmentPoints.size();sp++)
				points.add(segmentPoints.get(sp));
			
		}
		
		saveMap(points);
		
		ann = new ANNdata();
		this.vertices = vertices;
		
	}
	
	public void setMapPoints(ArrayList<double[]> points, double[][] vertices) {
		
		
		ArrayList<double[]> rewrite = new ArrayList<double[]>();
		
		for(int i=0;i<points.size();i++) {
			double[] curr = points.get(i);
			rewrite.add(new double[] {curr[0],curr[2]});
		}
				
		
		this.vertices = vertices;
		saveMap(rewrite);
		
	}
	
	public void setMapRandom() {}
	
	public void setTrajectories(String[] path){
		
		trajectories = new ArrayList<ANNdata>();
		
		for(int i=0; i<path.length;i++) {
			ANNdata annData=new ANNdata(path[i]);
			trajectories.add(annData);
		}
		
		this.ann = trajectories.get(0);
//		Main.ann = this.ann;
		
	}
	
	
	public void setRandomTrajectories(int count){
		
	}
	
	private void saveMap(ArrayList<double[]> points) {
		this.map = new double[points.size()][2];
		
		for(int sp=0;sp<points.size();sp++) {
			double x = points.get(sp)[0];
			double z = points.get(sp)[1];
			this.map[sp]=new double[] {x,0,z};
		}
		
		mapTransform = new Transform(this.map);
		
	}
}

class Segment{
	
	double xInterval, a, b, xA, yA, xB, yB;
	double DISTANCE_BETWEEN_CONSECUTIVE_POINTS = 0.1d;
	
	public Segment( double xA,double yA,double xB,double yB) {
		
		this.xA = xA;
		this.yA = yA;
		this.xB = xB;
		this.yB = yB;
		
		
		double dy = yB-yA;
		double dx = xB-xA;
		this.a=dy/dx;
		this.b=yA-xA*this.a;
		
		//xInterval for points probing should depend on the line slope
		double slope = Math.atan(this.a);
		this.xInterval = Math.cos(slope)*DISTANCE_BETWEEN_CONSECUTIVE_POINTS;
		if(xA>xB)this.xInterval=-this.xInterval;
		
	}
	
	public ArrayList<double[]> points(){
		ArrayList<double[]> generatedPoints = new ArrayList<double[]>();
		int numberOfBetweenPoints = (int)Math.floor(Math.abs(xA-xB)/Math.abs(xInterval));
		
		generatedPoints.add(new double[] {xA, yA});
		double x = xA;
		for(int p=0;p<numberOfBetweenPoints;p++) {
			x+=xInterval;
			double y = a*x+b;
			generatedPoints.add(new double[] {x, y});
		}
		generatedPoints.add(new double[] {xB, yB});
		return generatedPoints;
	}
}
