import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class ANNdata {
	
	ArrayList<double[]> points = null;
	ArrayList<double[]> shifts = null;
	double positionX=0;
	double positionY=0;
	double rotation=0;
	double multiplier = 1.1d;
	
	public ANNdata(String path) {
		
		ArrayList<double[]> instructions = new ArrayList<double[]>();
		points = new ArrayList<double[]>();
		shifts = new ArrayList<double[]>();
		
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
		
		//initial rotation
		this.rotation= instructions.get(0)[4];
		
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
	
	public void add(FeatureCloud nextCloud, int step) {
		
		double[] change = shifts.get(step-1);
		positionX+=change[0];
		positionY+=change[1];
		rotation+=change[2];
		Transform transform = new Transform(nextCloud);
		transform.rotate(0, 0, rotation);
		transform.translate(-positionX*0.9d, positionY*0.9d);
		addTransform(transform);
		
	}
	
	public void addTransform(Transform t) {
		for(int i=0;i<t.points.length;i++) {
			
			points.add(t.points[i]);
			
		}
	}

}


