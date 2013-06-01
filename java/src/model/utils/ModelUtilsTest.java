/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils;

/**
 *
 * @author woodstock
 */
import java.util.Random;
import java.util.Arrays;
import model.utils.files.FileIO;
import org.shared.array.*;

public class ModelUtilsTest {
    
    public static void neighVar(){
        
        int nofRows = 10;
        int nofCols = 10;
        RealArray input = new RealArray(nofRows,nofCols);
        
        for(int r=0; r<nofRows; ++r){
            
            for(int c=0; c<nofCols; ++c){
                
                input.set(r+c, r, c);
            }
        }
        System.out.printf("i =%n%s%n", input.toString());
        RealArray output = ModelUtils.calcNeighVar(input, 2);
        
        double min = output.aMin();
        output = output.uAdd(-min);
        double max = output.aMax();
        output = output.uMul(1.0/max);
        
        System.out.printf("n =%n%s%n", output.toString());
    }
    
     public static boolean argsort_double(){
        
        int n = 5;
        double [] input = new double[n];
        ModelUtils.rand(input);
        
        int [] indicies = ModelUtils.argsort(input);
        System.out.println(Arrays.toString(input));
        System.out.println(Arrays.toString(indicies));
        
        double [] input_sorted = Arrays.copyOf(input, n);
        Arrays.sort(input_sorted);
        System.out.println(Arrays.toString(input_sorted));
        
        boolean success = true;
        for(int i=0; i<n && success; ++i){
            
            success = input_sorted[indicies[i]] == input[i];
            assert input_sorted[indicies[i]] == input[i];
        }
        System.out.println(success);
        return success;
    }
     
    public static boolean argsort_int(){
        
        int nof_inputs = 5;
        int [] input = new int[ nof_inputs ];
        ModelUtils.rand(input);
        
        int [] indicies = ModelUtils.argsort(input);
        System.out.println(Arrays.toString(input));
        System.out.println(Arrays.toString(indicies));
        
        int [] input_sorted = Arrays.copyOf(input, nof_inputs);
        Arrays.sort(input_sorted);
        System.out.println(Arrays.toString(input_sorted));
        
        boolean success = true;
        for(int i=0; i<nof_inputs && success; ++i){
            
            success = input_sorted[indicies[i]] == input[i];
            assert input_sorted[indicies[i]] == input[i];
        }
        System.out.println(success);
        return success;
    }
    
    /**
     * @brief test argmax on double[] 
     * @return success
     */
    public static boolean argmax_double(){
        
        int n = 5;
        double [] input = new double[n];
        ModelUtils.rand(input);
        
        int argmax = ModelUtils.argmax(input);
        double max = input[argmax];
        
        Arrays.sort(input);
        boolean success = max == input[n-1];
        System.out.println(success);
        return success;
    }

    /**
     * @brief test argmax on int[] 
     * @return success
     */
    public static boolean argmax_int(){
        
        int n = 5;
        int [] input = new int[n];
        ModelUtils.rand(input);
        
        int argmax = ModelUtils.argmax(input);
        double max = input[argmax];
        
        Arrays.sort(input);
        boolean success = max == input[n-1];
        System.out.println(success);
        return success;
    }

    /**
     * @brief test argmin on double[] 
     * @return success
     */
    public static boolean argmin_double(){
        
        int n = 5;
        double [] input = new double[n];
        ModelUtils.rand(input);
        
        int argmin = ModelUtils.argmin(input);
        double min = input[argmin];
        
        Arrays.sort(input);
        boolean success = min == input[0];
        System.out.println(success);
        return success;
    }

    /**
     * @brief test argmax on int[] 
     * @return success
     */
    public static boolean argmin_int(){
        
        int n = 5;
        int [] input = new int[n];
        ModelUtils.rand(input);
        
        int argmin = ModelUtils.argmin(input);
        double min = input[argmin];
        
        Arrays.sort(input);
        boolean success = min == input[0];
        System.out.println(success);
        return success;
    }
    
    public static void nextPow2(){
        
        Random generator;
        generator = new Random();
        
        int nof_trials = 10;
        for(int i=0; i<nof_trials; i++){
            
            int n = generator.nextInt()%100;
            int nextPow = ModelUtils.nextPow(n, 2);
            System.out.println(n+" "+nextPow);
        }
    }   
    
    public static void pad2(){
        
        int nofRows = 3;
        int nofCols = 3;
        int nGuardBandWidth = 2;
        int [][] x = new int [ nofRows ][ nofCols ];
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                x[r][c] = r+c;
            }
        }

        System.out.println("x");
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                System.out.print(x[r][c] + "  ");
            }
            System.out.println();
        }
        
        int [][] y;
        y = ModelUtils.pad2D(x, nGuardBandWidth, ModelUtils.MODE_PAD_ROWS_BOTTOM);
        System.out.println("rows");
        System.out.println(y.length);
        System.out.println(y[0].length);
        for(int r=0; r<nofRows+nGuardBandWidth; r++){
            
            for(int c=0; c<nofCols; c++){
                
                System.out.print(y[r][c] + "  ");
            }
            System.out.println();
        }
        
        y = ModelUtils.pad2D(x, nGuardBandWidth, ModelUtils.MODE_PAD_COLS_RIGHT);
        System.out.println("cols");
        System.out.println(y.length);
        System.out.println(y[0].length);
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols+nGuardBandWidth; c++){
                
                System.out.print(y[r][c] + "  ");
            }
            System.out.println();
        }
        
        y = ModelUtils.pad2D(x, nGuardBandWidth, ModelUtils.MODE_PAD_BOTH_BOTTOMRIGHT);

        System.out.println("both");
        System.out.println(y.length);
        System.out.println(y[0].length);
        for(int r=0; r<nofRows+nGuardBandWidth; r++){
            
            for(int c=0; c<nofCols+nGuardBandWidth; c++){
                
                System.out.print(y[r][c] + "  ");
            }
            System.out.println();
        }
        
    }

    public static void pad3(){
        
        int nofRows = 3;
        int nofCols = 3;
        int nGuardBandWidth = 2;
        RealArray x = new RealArray(nofRows,nofCols);
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                x.set(r+c,r,c);
            }
        }
        System.out.printf("x =%n%s%n", x.toString());
        
        //RealArray y = pad2D(x, nGuardBandWidth, MODE_PAD_BOTH_BOTTOMRIGHT);
        RealArray y = ModelUtils.pad2D(x, nGuardBandWidth, ModelUtils.MODE_PAD_BOTH_SURROUND);
        //RealArray y = pad2D(x, nGuardBandWidth, MODE_PAD_COLS_LEFT);
        //RealArray y = pad2D(x, nGuardBandWidth, MODE_PAD_COLS_RIGHT);
        //RealArray y = pad2D(x, nGuardBandWidth, MODE_PAD_ROWS_BOTTOM);
        //RealArray y = pad2D(x, nGuardBandWidth, MODE_PAD_ROWS_TOP);
        
        System.out.printf("y =%n%s%n", y.toString());
        
        RealArray z0 = ModelUtils.pad2D(y, -nGuardBandWidth, ModelUtils.MODE_PAD_BOTH_SURROUND);
        
        System.out.printf("z0 =%n%s%n", z0.toString());
        
        //RealArray z1 = pad2D(z0, -nGuardBandWidth, MODE_PAD_COLS_RIGHT);
        //System.out.printf("z =%n%s%n", y.toString());
    }
    
    public static void extract_column(){
        
        int nofRows = 3;
        int nofCols = 3;
        int [][] x = new int [ nofRows ][ nofCols ];
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                x[r][c] = r+c;
            }
        }
        x[0][2]=100;
        int nColumnOfInterest;
        int [] y;
        nColumnOfInterest = 0;
        y = ModelUtils.extract_columns(x, nColumnOfInterest);
        
        System.out.println("x");
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                System.out.print(x[r][c] + "  ");
            }
            System.out.println();
        }
        
        System.out.print("column of interest" + nColumnOfInterest + ": ");  
        for(int c=0; c<nofCols; c++){
                
                System.out.print(y[c] + "  ");
        }

        nColumnOfInterest = 2;
        y = ModelUtils.extract_columns(x, nColumnOfInterest);

        System.out.print("column of interest" + nColumnOfInterest + ": ");        
        for(int c=0; c<nofCols; c++){
               
            System.out.print(y[c] + "  ");
        }
    }
    
    public static void stopwatch(){
        
        Stopwatch stopwatch;
        stopwatch = new Stopwatch();
        
        stopwatch.start();
        
        int sum = 0;
        int n = 1000000;
        for(int i=0; i<n; i++){
            
            sum += i;
        }
        try{
            Thread.sleep(2000);//sleep for 1000 ms
        }catch(Exception e){
            
        }
        stopwatch.stop();
        System.out.println(stopwatch.getElapsedTime());
    }
    
}
