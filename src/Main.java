import java.awt.Canvas;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferStrategy;
import java.awt.image.BufferedImage;
import java.awt.image.ColorConvertOp;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

public class Main {
	
	static int height;
	static int width;
	static String title;
	static JFrame frame;
	static Canvas canvas;
	static BufferStrategy bufferStrategy;
	static Graphics graphics;
	
	static String FILENAME_PREFIX = "t";
	static String PATH = "C:\\Users\\Lenovo\\Desktop\\dane\\tests\\test_A\\screenshots\\trajectory1\\";
	static int NUMBER_OF_POSES = 144;
	static int[][][][] guassians = null;
	
	static ImageSIFT A=null,B=null;
	static FeatureCloud FC=null;
	static FeatureCloud previous=null;
	static SimpleICP drawICP = null;
	public static ANNdata ann = null;
	public static Simulator sim, sim2;

	
	
	
	public static void main(String[] args) {

		initiliseUI();
		
		//generateTrajectories(10,20);
		//processPhotos();
		//GenerateNeuralNetData();
		ICP();

		
	}
	

	
	static void ICP(){
		
		KalmanFilter filterA = new KalmanFilter();
		
		ann=new ANNdata("C:\\Users\\Lenovo\\Desktop\\dane\\tests\\test_A\\trajectories\\trajectory1\\t.txt");
		for(int loop=1;loop<=NUMBER_OF_POSES;loop++) {
		FeatureCloud cloud = new FeatureCloud("C:\\Users\\Lenovo\\Desktop\\dane\\tests\\test_A\\screenshots\\trajectory1\\Cloud"+loop+".txt");
		FC=cloud;
		ann.add(cloud, loop);
		
		if(previous!=null) {
			SimpleICP icp = new SimpleICP(previous, cloud, false);
			drawICP = icp;
			double[] shift = filterA.filter(icp.RANSAC());
			
			System.out.println("X: "+shift[0]);
			System.out.println("Z: "+shift[1]);
			System.out.println("Rotation: "+shift[2]);
			
			try {TimeUnit.MILLISECONDS.sleep(1200);}catch(Exception exc) {}
		}
		previous=FC;
		}
	}
	
	static void processPhotos() {
		guassians=generateGuassians();
		for(int loop=1;loop<=NUMBER_OF_POSES;loop++) {
			
			BufferedImage Lbi = loadFromFile(PATH+FILENAME_PREFIX+loop+'l'+".PNG");
			BufferedImage Rbi = loadFromFile(PATH+FILENAME_PREFIX+loop+'r'+".PNG");
	
			ImageSIFT LI = new ImageSIFT(Lbi,5,guassians);
			ImageSIFT RI = new ImageSIFT(Rbi,5,guassians);
			B=RI;
			A=LI;
			FeatureCloud cloud = new FeatureCloud(LI,RI);
			FC=cloud;
			saveData(FC, loop);
		}
	}
	
	static void generateTrajectories(int points,int count) {
		sim = new Simulator(true,false,false);
		sim.setMapVerticies(new double[][] {
			{2.87, 1.81},
			{12.58, 2.75},
			{16.11, 4.61},
			{13.19, 11,37},
			{14.04, 11.83},
			{17.04, 5.05},
			{21.52, 7.27},
			{29.63, 6.43},
			{33.5, 11},
			{32.98, 20.37},
			{23.08, 23.43},
			{15.67, 30.83},
			{5.4, 25.4},
			{5.36, 12.48},		
		});
	for(int i=1;i<=count;i++)
	saveTrajectory(sim.generateRandomTrajectory(points),"C:\\Users\\Lenovo\\Desktop\\dane\\tests\\test_A\\trajectories\\trajectory"+i+"\\","t");
	}
	
	static void GenerateNeuralNetData() {
		ArrayList<String> samples= new ArrayList<String>();
		String sample="";
		for(int f=0;f<5000;f++) {
			
			sim = new Simulator(true,false,false);
			sim.setMapVerticies(new double[][] {
				{2.87, 1.81},
				{12.58, 2.75},
				{16.11, 4.61},
				{13.19, 11,37},
				{14.04, 11.83},
				{17.04, 5.05},
				{21.52, 7.27},
				{29.63, 6.43},
				{33.5, 11},
				{32.98, 20.37},
				{23.08, 23.43},
				{15.67, 30.83},
				{5.4, 25.4},
				{5.36, 12.48},		
			});
			
			
		
		saveTrajectory(sim.generateRandomTrajectory(2),"C:\\Users\\Lenovo\\Desktop\\dane\\trajectories\\Room A\\","temp");
		sim.setTrajectories(new String[] {"C:\\Users\\Lenovo\\Desktop\\dane\\trajectories\\Room A\\temp.txt"});
		for(int z=0;z<sim.ann.shifts.size();z++) {
			sim.simulateOneStep();
			//try {TimeUnit.MILLISECONDS.sleep(100);}catch(Exception exc) {}
		}
		
		samples.add(sim.ann.fileBuffer);
		System.out.println(f);
		}
		
		
		
		saveANNsamples(samples,"C:\\Users\\Lenovo\\Desktop\\dane\\ANN samples\\","ANN_A");
		
	}
	
	static void saveANNsamples(ArrayList<String> ANNsamples, String directoryPath, String filename) {
		 try {
			 FileWriter writer = new FileWriter(directoryPath+filename+".txt");
		      // Creates a BufferedWriter
		      BufferedWriter output = new BufferedWriter(writer);

			 	
			 	for(int i=0;i<ANNsamples.size();i++) {
			 		String current = ANNsamples.get(i);
			 		output.write(current);
			 		output.write("\r\n");
			 		 
			 	}
			      // Closes the writer
			      output.close();
	            writer.close();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
	}
	
	static void saveTrajectory(String trajectory, String directoryPath, String filename) {
		 try {
			 	PrintWriter writer = new PrintWriter(directoryPath+filename+".txt");
	            writer.write(trajectory);
	            writer.close();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
	}
	
	static void saveData(FeatureCloud cloud, int oridinalName){
	       try {
	            FileWriter writer = new FileWriter("C:\\Users\\Lenovo\\Desktop\\dane\\tests\\test_A\\screenshots\\trajectory1\\Cloud"+oridinalName+".txt", true);
	            String currentFeature="";
	            for(int i = 0; i<cloud.featurePositions.size();i++)
	            {
	            	for(int f=0;f<128;f++) {
	            		currentFeature+=cloud.features.get(i)[f]+";";
	            	}
	            	currentFeature+=cloud.featurePositions.get(i)[0]+";";
	            	currentFeature+=cloud.featurePositions.get(i)[1]+";";
	            	currentFeature+=cloud.featurePositions.get(i)[2]+";";
	            	writer.write(currentFeature);
		            writer.write("\r\n");
		            currentFeature="";
	            }
	            writer.write("\r\n");  
	            
	            writer.close();
	        } catch (IOException e) {
	            e.printStackTrace();
	        }
	 
	}
	
	static void draw() {
		graphics.setColor(Color.WHITE);
		graphics.fillRect((int)0, (int)0, 960*2, 520*2);
		
		if(A!=null) {
		DrawImage(A,0,0);
		//DrawAllFeatures(A,0,0);
		}
		
		if(B!=null) {
			DrawImage(B,960,0);
			//DrawAllFeatures(A,0,0);
			}
		

//		if(FC!=null) {
//			//DrawCloudPlane(FC,0,0);
//			DrawCloud(FC,960,800);
//		}
		
		if(drawICP!=null) {
			drawICP(drawICP,500,500);
		}
		
//		if(sim!=null&&sim.ann!=null) {
//			drawANNdata(sim.ann,500,600);
//		}
		
		if(sim2!=null&&sim2.ann!=null) {
			drawANNdata(sim2.ann,1400,600);
		}
		
	}
	
	static void drawANNdata(ANNdata ann, double Ox, double Oy) {
		
		for(int i=0;i<ann.points.size();i++) {
        	double x = ann.points.get(i)[0];
        	double z = -ann.points.get(i)[2];
        	graphics.setColor(Color.BLUE);
        	graphics.fillOval((int)(Ox+x*20), (int)(Oy+z*20), (int)(5), (int)(5));	        	
     }
		for(int r=0;r<ann.rays.size();r++) {
			double angle = ann.rays.get(r).angle;
			double distance = ann.rays.get(r).distance;
			graphics.drawLine((int)(Ox+0), (int)(Oy+0), (int)(Ox+20*-distance*Math.cos((angle/360d)*2*Math.PI)), (int)(Oy+20*-distance*Math.sin((angle/360d)*2*Math.PI)));  
			
		}
		
	}
	
	static void drawICP(SimpleICP icp, double Ox, double Oy) {
		
		
		 for(int i=0;i<icp.prev.points.length;i++) {
	        	double x = icp.prev.points[i][0];
	        	double z = -icp.prev.points[i][2];
	        	graphics.setColor(Color.BLUE);
	        	graphics.fillOval((int)(Ox+x*20), (int)(Oy+z*20), (int)(5), (int)(5));	        	
	     }
		 
		 for(int i=0;i<icp.curr.points.length;i++) {
	        	double x = icp.curr.points[i][0];
	        	double z = -icp.curr.points[i][2];
	        	graphics.setColor(Color.GREEN);
	        	graphics.fillOval((int)(Ox+x*20), (int)(Oy+z*20), (int)(5), (int)(5));	        	
	     }
		 
		//prevIndex,prevX,prevZ,,currIndex,currX,currZ,distance
		 for(int i=0;i<icp.featureMatches.size();i++) {
	        	double x1 = icp.featureMatches.get(i)[1]*20;
	        	double y1 = -icp.featureMatches.get(i)[2]*20;
	        	double x2 = icp.featureMatches.get(i)[4]*20;
	        	double y2 = -icp.featureMatches.get(i)[5]*20;
	        	
	        	graphics.setColor(Color.RED);
	        	graphics.drawLine((int)(Ox+x1), (int)(Oy+y1), (int)(Ox+x2), (int)(Oy+y2));        	
	     }
		 
	}
	
	static void DrawImage(ImageSIFT sift, double Ox, double Oy) {
		
		  double highest = 0;
	        double lowest = 0;
	        for(int x=0;x<sift.original.length;x++) 
	            for(int y=0;y<sift.original[0].length;y++) {
		        	
		        	double value = sift.original[x][y];
		        	if (value>highest)highest=value;
		        	if (value<lowest)lowest=value; 	
		        }
	    double fullRange = Math.abs(highest-lowest);
		for(int x=0;x<sift.original.length;x++) 
        for(int y=0;y<sift.original[0].length;y++) {
        	
        	long value = (long)((sift.original[x][y]-lowest)/(double)fullRange*255d);
        	graphics.setColor(new Color((int)(value),(int)(value),(int)(value)));
        	graphics.drawLine((int)(Ox+x), (int)(Oy+y) ,(int)(Ox+x),(int)(Oy+y));
       
        }
			
		
	}
	
	static void DrawCloudPlane(FeatureCloud cloud, double Ox, double Oy) {

		
		 for(int i=0;i<cloud.featurePositions.size();i++) {
	        	
	        	double x = cloud.featurePositions.get(i)[0];
	        	double y = cloud.featurePositions.get(i)[1];
	        	double z = cloud.featurePositions.get(i)[3];
	        	
	
	        	long zmul = (long)(z);
	        	
	        	graphics.setColor(Color.BLUE);

	        	if(zmul>0&&zmul<50)
	        	graphics.fillOval((int)(Ox+x), (int)(Oy+y), (int)(zmul), (int)(zmul));
	        	
	     }
			
		
	}
	
	static void DrawCloud(FeatureCloud cloud, double Ox, double Oy) {



		 for(int i=0;i<cloud.featurePositions.size();i++) {
	        	
	        	double x = cloud.featurePositions.get(i)[0];
	        	double h = -cloud.featurePositions.get(i)[2];
	        	long value = (long)(h);
	        	
	        	//graphics.setColor(Color.BLUE);
	

	        	graphics.setColor(new Color((int)(200),(int)(0),(int)(100)));
	        	graphics.fillOval((int)(Ox+x*20), (int)(Oy+h*20), (int)(-value*1d), (int)(-value*1d));

	        	
	     }
			
		
	}
	
	static void DrawAllFeatures(ImageSIFT sift, int Ox, int Oy){
		

        for(int i=0;i<sift.featurePositions.size();i++) {
        	graphics.setColor(Color.CYAN);
        	int x = (int)sift.featurePositions.get(i)[0];
        	int y = (int)sift.featurePositions.get(i)[1];
        	graphics.drawLine((Ox+x), (Oy+y) ,(Ox+x),(Oy+y));
       
        }
		
	}
	
	static int[][][][] generateGuassians(){
		int[][][] guassianLibrary = ImageSIFT.initialiseGuassianLibrary(7, 5, 1.6d, Math.sqrt(2)); 
	    int[][][] CircularGuassianLibrary = ImageSIFT.initialiseCircularGuassian(16, 2, 1.6d*1.5, Math.sqrt(2));
		
	    return new int[][][][] {guassianLibrary,CircularGuassianLibrary};
	}
	
	static BufferedImage loadFromFile(String path) {
		
		BufferedImage img=null;
		 try {img = ImageIO.read(new File(path));} 
		    catch (IOException e) {System.out.print(e);}
			BufferedImage gray = new BufferedImage(img.getWidth(),img.getHeight(),BufferedImage.TYPE_BYTE_GRAY);
		    ColorConvertOp op = new ColorConvertOp(img.getColorModel().getColorSpace(),gray.getColorModel().getColorSpace(),null);
		    op.filter(img,gray);
		    
		 return gray;
	}

	static void initiliseUI() {
		
	    title = "Test Window";
	    width = 1920;
	    height =width*9/16;
	    frame = new JFrame(title);
	    frame.setSize(width, height);
	    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    frame.setLocationRelativeTo(null);
	    frame.setResizable(false);
	    frame.setVisible(true);
	    canvas = new Canvas();
	    canvas.setSize(width, height);
	    canvas.setBackground(Color.BLACK);
	    canvas.setVisible(true);
	    canvas.setFocusable(false);
	    frame.add(canvas);
	    canvas.createBufferStrategy(3);
	    Thread t1 = new Thread(new Concurrently());
	    t1.start();

	}

	static class KalmanFilter{
		double[] KalmanGain = {0d,0d,0d};
		double[] EstimationError ={1d,1d,1d};
		double[] MeasurementError ={1d,1d,1d};
		double[] previousEstimate ={0d,0d,0d};
		double[] currentEstimate ={0d,0d,0d};
		double[] NextEstimate ={0d,0d,0d};
		
		double[] filter(double[] measurement){
			
			
			for(int i =0;i<3;i++) {
				KalmanGain[i]=EstimationError[i]/(EstimationError[i]+MeasurementError[i]);
				currentEstimate[i]=previousEstimate[i]+(KalmanGain[i]*(measurement[i]-previousEstimate[i]));
				EstimationError[i]=(1-KalmanGain[i])*EstimationError[i];
				previousEstimate[i]=currentEstimate[i];
			}
			

			return new double[] {currentEstimate[0],currentEstimate[1],currentEstimate[2]};
		}
	} 
	
	static class Concurrently implements Runnable {
		public void run(){
		
			while(true) {
	        bufferStrategy = canvas.getBufferStrategy();
	        graphics = bufferStrategy.getDrawGraphics();
	        graphics.clearRect(0, 0, width, height);
	        
	        draw();
	        
	        bufferStrategy.show();
	        graphics.dispose();
			}
		   } 
		}
}
