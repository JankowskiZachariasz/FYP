import java.awt.Canvas;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferStrategy;
import java.awt.image.BufferedImage;
import java.awt.image.ColorConvertOp;
import java.io.File;
import java.io.IOException;
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
	
	static String FILENAME_PREFIX = "t1_";
	static String PATH = "C:\\Users\\Lenovo\\Desktop\\dane\\";
	static int NUMBER_OF_POSES = 45;
	static int[][][][] guassians = null;
	
	static ImageSIFT A=null,B=null;
	static FeatureCloud FC=null;

	
	
	
	public static void main(String[] args) {
		guassians=generateGuassians();
		initiliseUI();

		
		for(int loop=1;loop<=NUMBER_OF_POSES;loop++) {
			
			BufferedImage Lbi = loadFromFile(PATH+FILENAME_PREFIX+loop+'r'+".PNG");
			BufferedImage Rbi = loadFromFile(PATH+FILENAME_PREFIX+loop+'l'+".PNG");
		
			ImageSIFT LI = new ImageSIFT(Lbi,5,guassians);
			A=LI;
			ImageSIFT RI = new ImageSIFT(Rbi,5,guassians);
			B=RI;
			
			FeatureCloud cloud = new FeatureCloud(LI,RI);
			
			FC=cloud;
			
			
			
		}
		
//		System.out.println(FC.featurePositions.size());
//		
//		for(int f=0;f<FC.featurePositions.size();f++)
//			System.out.println(FC.featurePositions.get(f)[2]);
		
	}
	static void draw() {
		graphics.setColor(Color.WHITE);
		graphics.fillRect((int)0, (int)0, 960*2, 520*2);
		
		if(A!=null) {
			DrawImage(A,0,0);
		//DrawAllFeatures(A,0,0);
		}
		

		if(FC!=null) {
			DrawCloudPlane(FC,0,0);
			//DrawCloud(FC,0,0);
			
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

		  double highest = 10;
	        double lowest = -10;
		double fullRange = Math.abs(highest-lowest);
		
		 for(int i=0;i<cloud.featurePositions.size();i++) {
	        	
	        	double x = cloud.featurePositions.get(i)[0];
	        	double y = cloud.featurePositions.get(i)[1];
	        	double h = cloud.featurePositions.get(i)[2];
	        	double z = cloud.featurePositions.get(i)[3];
	        	
	        	long value = (long)(h);
	        	long zmul = (long)(z);
	        	
	        	graphics.setColor(Color.BLUE);
	        	//graphics.setColor(new Color((int)(250),(int)(0),(int)(value)));
	        	if(zmul>0&&zmul<50)
	        	graphics.fillOval((int)(Ox+x), (int)(Oy+y), (int)(zmul), (int)(zmul));
	        	//graphics.drawLine((int)(Ox+x*10), (int)(Oy+y*10) ,(int)(Ox+x*10),(int)(Oy+y*10));
	        	
	     }
			
		
	}
	
	static void DrawCloud(FeatureCloud cloud, double Ox, double Oy) {

		  double highest = 30;
	        double lowest = 0;
//		for(int i=0;i<cloud.featurePositions.size();i++) {
//			double value = cloud.featurePositions.get(i)[2];
//        	if (value>highest)highest=value;
//        	if (value<lowest)lowest=value; 	
//		}
		double fullRange = Math.abs(highest-lowest);
		graphics.setColor(Color.WHITE);
		graphics.fillRect(0, 0, 960, 520);
		 for(int i=0;i<cloud.featurePositions.size();i++) {
	        	
	        	double x = cloud.featurePositions.get(i)[0];
	        	double h = cloud.featurePositions.get(i)[2];
	        	double y = cloud.featurePositions.get(i)[1];
	        	
	        	long value = (long)(h);
	        	
	        	//graphics.setColor(Color.BLUE);
	        	value=(long)((h-lowest)/(double)fullRange*255d);
	        	value=255-value;
	        	graphics.setColor(new Color((int)(value),(int)(0),(int)(value)));
	        	graphics.fillOval((int)x, (int)y, (int)(value*0.04d), (int)(value*0.04d));
	        	graphics.drawLine((int)(Ox+x), (int)(Oy+y) ,(int)(Ox+x),(int)(Oy+y));
	        	
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
