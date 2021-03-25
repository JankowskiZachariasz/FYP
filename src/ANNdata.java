import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

//this class can only afford one trajectory
public class ANNdata {
	
	int NUMBER_OF_RAYS = 10;
	
	
	ArrayList<double[]> points = null;
	ArrayList<Ray> rays = null;
	ArrayList<double[]> shifts = null;
	
	String fileBuffer="";
	double positionX=0;//if set to 0 it means that the rat starts at 0,0 - which is only correct if it really does or
	double positionY=0;//if we want the ANN to learn the general SLAM (with positions relative to the starting point)
	double rotation=0;
	double multiplier = 1.0d;
	
	public ANNdata( ){
	
		points = new ArrayList<double[]>();
		shifts = new ArrayList<double[]>();
		rays = createRays();
		
	}
	
	public ANNdata(String path) {
		
		ArrayList<double[]> instructions = new ArrayList<double[]>();
		points = new ArrayList<double[]>();
		shifts = new ArrayList<double[]>();
		rays = createRays();
		
		try {
			FileReader reader = new FileReader(path);
			BufferedReader bufferedReader = new BufferedReader(reader);
			
			String line;
			
			while ((line = bufferedReader.readLine()) != null) {
			
				
				String[] doubles = line.split(",");
				double[] add = new double[7];
				add[0]=-Double.parseDouble(doubles[0]);
				add[1]=Double.parseDouble(doubles[1]);
				add[2]=Double.parseDouble(doubles[2]);
				add[3]=Double.parseDouble(doubles[3]);
				add[4]=Double.parseDouble(doubles[4]);
				add[5]=Double.parseDouble(doubles[5]);
				add[6]=Double.parseDouble(doubles[6]);
				instructions.add(add); 
				
			}
			reader.close();
		} catch (IOException e) {
		e.printStackTrace();
		}
		
		//initial rotation and position
		this.rotation= instructions.get(0)[4];
		this.positionX= instructions.get(0)[2];
		this.positionY= instructions.get(0)[0];
		
		
		for(int y =1; y<instructions.size();y++) {
			double[] previous = instructions.get(y-1);
			double[] current = instructions.get(y);
			
			
			double dx=current[2]-previous[2];
			double dz=current[0]-previous[0];
			
			double dr=current[4]-previous[4];
			double steps = current[6];
			
			for(int j=0;j<(int)steps;j++) {
				
				shifts.add(new double[] {dx/steps, dz/steps,dr/steps});
			}
		}
		
	}
	
	public void setTrajectory(ArrayList<double[]> shifts){
		this.shifts = shifts;
	}
	
	
	private ArrayList<Ray> createRays(){
		
		ArrayList<Ray> basket = new ArrayList<Ray>();
		double startingAngle = 29.267007901d;//+4
		double completeViewAngle = 121.465984198d;//-8
		
		double currentAngle = startingAngle;
		for(int i=0; i<NUMBER_OF_RAYS;i++) {
			basket.add(new Ray(currentAngle));
			currentAngle+=(completeViewAngle/(NUMBER_OF_RAYS-1));
		}
		
		return basket;
	}
	
	public void add(FeatureCloud nextCloud, int step) {
		
		double[] change = shifts.get(step-1);
		positionX+=change[0];
		positionY+=change[1];
		rotation+=change[2];
		Transform transform = new Transform(nextCloud);
		
		transform.translate(-positionY, positionX);
		transform.rotate(-positionY, positionX, rotation);
		addTransform(transform);
		
	}
	
	public void generateAnnStep(Transform t, int step,boolean camView) {
		
		
		String s="";
		double[] change = shifts.get(step-1);
		positionY+=change[1];
		positionX+=change[0];
		rotation+=change[2];
		for(int r =0;r<NUMBER_OF_RAYS;r++) {
			s+=rays.get(r).distance(t,camView)+",";
			
		}
		//System.out.println(s);
		
//		s+=-change[1]+",";
//		s+=change[0]+",";
//		s+=change[2]+",";
		
		s+=-positionY+",";
		s+=positionX+",";
		s+=rotation+",";
		s+="\r\n";
		fileBuffer+=s;
		
		setTransform(t);
		
		
	}
	
	public void saveTrajectory() {}
	
	public void setTransform(Transform t) {
		points = new ArrayList<double[]>();
		for(int i=0;i<t.points.length;i++) {
			
			points.add(t.points[i]);
			
		}
	}
	
	public void addTransform(Transform t) {
		for(int i=0;i<t.points.length;i++) {
			
			points.add(t.points[i]);
			
		}
	}

}

class Ray{
	double angle, a,distance;
	final double DISTANCE_THRESHOLD = 0.5d;
	
	public Ray(double angle) {
			
		this.a = -Math.tan((angle/360)*2*Math.PI);
		this.angle = angle;
	}
	
	public double distance(Transform t,boolean camView) {
		
		double[] histogram = new double[300];
		for(int i =0; i<t.points.length;i++ ) {
			int index = distance(t.points[i][0],t.points[i][2]);
			if(index!=-1) {
				
				if(index>299)index=299;
				histogram[index]+=1;
			}
			
			
		}
		double largest=-1;
		int index =-1;
		
		if(camView)
		for(int v=0;v<300;v++) {
			if(histogram[v]>=largest) {
				largest=histogram[v];
				index = v;
			}
		}
		else
		for(int v=299;v>=0;v--) {
			if(histogram[v]>=2) {
				largest=histogram[v];
				index = v;
			}
		}
		
		
		
		this.distance= (double)index/10d;
		return (double)index/10d;
	}
	
	private int distance(double x, double z) {
		
		if(z<0)return -1;
		
		double lineDistance = Math.abs(a*x-1*z)/Math.sqrt(a*a+1);
		
		if(lineDistance<=DISTANCE_THRESHOLD) {
			double centerDistance = Math.sqrt(x*x+z*z);
			if (centerDistance>30) centerDistance=30d;
			return (int)Math.round(centerDistance*10);
			
		}
		else return -1;
		
		//-1 means that a point isn't in close proximity to a line
	}
	
	
}


