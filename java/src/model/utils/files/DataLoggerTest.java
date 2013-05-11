/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;
import java.util.Random;
import java.io.File;
import java.util.Scanner;
import java.util.Arrays;

/**
 *
 * @author woodstock
 */
public class DataLoggerTest {
    
    public static int test_double(){
        
        Random generator = new Random();
        int nof_rows = 11;
        int nof_cols = 3;
        int cache_size = 2;
        String filepath = "tmp.csv";
        DataLogger data_logger = new DataLogger();
        data_logger.set_params(filepath, nof_cols, cache_size, true, false);
        data_logger.init();
        
        double [][] reference = new double[nof_rows][nof_cols];
        
        for(int i=0; i<nof_rows; ++i){
            
            double [] row = new double[nof_cols];
            for(int j=0; j<nof_cols; ++j){
                
                double v = generator.nextDouble();
                row[j] = v;
                reference[i][j] = v;
            }
            data_logger.add_sample(row);
        }
        
        data_logger.flush();
        
        // verify
        double [][] values = new double[nof_rows][nof_cols];
        File file   = new File(filepath);
        
        try(Scanner scanner = new Scanner(file)){
            
            scanner.useDelimiter("\n");
            String str_line;
            String [] arr_line_elements;
            double value;
            
            for(int i=0; i<nof_rows && scanner.hasNext(); ++i){

                str_line = scanner.next();
                str_line = str_line.replaceAll("\r", "");
                arr_line_elements = str_line.split(",");
                for(int j=0; j<nof_cols; ++j){
            
                    value = Double.parseDouble(arr_line_elements[j]);
                    values[i][j] = value;
                }
            }
            scanner.close();
        }
        catch(Exception e){
            
            System.err.println("Error: " + e.getMessage());
            return 1;
        }
        
        for(int i=0; i<nof_rows; ++i){
        
            System.out.println("r"+i+Arrays.toString(reference[i]));
            System.out.println("v"+i+Arrays.toString(values[i]));
            boolean is_equal = Arrays.equals(reference[i], values[i]);
            System.out.println("equal? "+is_equal);
            if(!is_equal)
                return 1;
        }
        
        return 0;
    }
    
    public static int test_int(){
        
        Random generator = new Random();
        int nof_rows = 11;
        int nof_cols = 3;
        int cache_size = 2;
        String filepath = "tmp.csv";
        DataLoggerInt data_logger = new DataLoggerInt();
        data_logger.set_params(filepath, nof_cols, cache_size);
        data_logger.init();
        
        int [][] reference = new int[nof_rows][nof_cols];
        
        for(int i=0; i<nof_rows; ++i){
            
            int [] row = new int[nof_cols];
            for(int j=0; j<nof_cols; ++j){
                
                int v = generator.nextInt();
                row[j] = v;
                reference[i][j] = v;
            }
            data_logger.add_sample(row);
        }
        
        data_logger.flush();
        
        // verify
        int [][] values = new int[nof_rows][nof_cols];
        File file   = new File(filepath);
        
        try(Scanner scanner = new Scanner(file)){
            
            scanner.useDelimiter("\n");
            String str_line;
            String [] arr_line_elements;
            int value;
            
            for(int i=0; i<nof_rows && scanner.hasNext(); ++i){

                str_line = scanner.next();
                str_line = str_line.replaceAll("\r", "");
                arr_line_elements = str_line.split(",");
                for(int j=0; j<nof_cols; ++j){
            
                    value = Integer.parseInt(arr_line_elements[j]);
                    values[i][j] = value;
                }
            }
            scanner.close();
        }
        catch(Exception e){
            
            System.err.println("Error: " + e.getMessage());
            return 1;
        }
        
        for(int i=0; i<nof_rows; ++i){
        
            System.out.println("r"+i+Arrays.toString(reference[i]));
            System.out.println("v"+i+Arrays.toString(values[i]));
            boolean is_equal = Arrays.equals(reference[i], values[i]);
            System.out.println("equal? "+is_equal);
            if(!is_equal)
                return 1;
        }
        
        return 0;
    }
}
