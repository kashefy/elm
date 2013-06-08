/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

/**
 *
 * @author woodstock
 */
import java.io.*;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collections;
import org.shared.array.*;

import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.core.CvType;

public class FileIO {
    
    public static String DIR_NAME_TUNE = "tune";
    public static String DIR_NAME_ACTIVITY_LAYER_F = "activity_layerF";
    public static String DIR_NAME_ACTIVITY_LAYER_Z = "activity_layerZ";
    public static String DIR_NAME_ATTENTION = "att";
    public static String DIR_NAME_WATCH = "watch";
    public static String DIR_NAME_CONFIG = "config";
    public static String FILTER_RESPONSE = "filterResponse";
    
    public static int DIM_INDEX_ROWS = 0;
    public static int DIM_INDEX_COLS = 1;
    
    // save matrix to image. Needs debugging. Does not store pixel values in correct intensity range.
    public static void saveArrayToImg(double[] par_values, int par_width, int par_height, String par_str_output_filepath){
        
        Mat src = new Mat(par_height, par_width, CvType.CV_64FC1);
        src.put(0, 0, par_values);
        Mat dst = new Mat(par_height, par_width, CvType.CV_8UC1);
        src.convertTo(dst, CvType.CV_8UC1, 255, 0);
        Highgui.imwrite(par_str_output_filepath, dst);
    }
    
    // save matrix of doubles to a text file/CSV. Needs debugging. Does not store pixel values in correct intensity range.
    public static void saveArrayToCSV(double[] par_arrayValues, int par_nofRows, int par_nofCols, String par_strOutputFilePath){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath))){
             
             for(int i=0; i<par_nofRows; i++){
                 
                 int rowOffset = i*par_nofCols;
                 for(int j=0;j<par_nofCols; j++){

                     pw.print(par_arrayValues[rowOffset+j]);
                     if (j<par_nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
                 
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    // save matrix of doubles to a text file/CSV. Needs debugging. Does not store pixel values in correct intensity range.
    public static void saveArrayToCSV(double[] par_arrayValues, int par_nofRows, int par_nofCols, String par_strOutputFilePath, boolean par_bAppend){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath, par_bAppend),par_bAppend)){
             
             for(int i=0; i<par_nofRows; i++){
                 
                 int rowOffset = i*par_nofCols;
                 for(int j=0;j<par_nofCols; j++){

                     pw.print(par_arrayValues[rowOffset+j]);
                     if (j<par_nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
                 
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    public static void saveArrayToCSV(float[] par_arrayValues, int par_nofRows, int par_nofCols, String par_strOutputFilePath, boolean par_bAppend){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath, par_bAppend),par_bAppend)){
             
             for(int i=0; i<par_nofRows; i++){
                 
                 int rowOffset = i*par_nofCols;
                 for(int j=0;j<par_nofCols; j++){

                     pw.print(par_arrayValues[rowOffset+j]);
                     if (j<par_nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
                 
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }

    // save matrix of integers to a text file/CSV. Needs debugging. Does not store pixel values in correct intensity range.
    public static void saveArrayToCSV(int[] par_arrayValues, int par_nofRows, int par_nofCols, String par_strOutputFilePath){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath))){
             
             for(int i=0; i<par_nofRows; i++){
                 
                 int rowOffset = i*par_nofCols;
                 for(int j=0;j<par_nofCols; j++){

                     pw.print(par_arrayValues[rowOffset+j]);
                     if (j<par_nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
                 
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    public static void saveArrayToCSV(int[] par_arrayValues, int par_nofRows, int par_nofCols, String par_strOutputFilePath, boolean par_bAppend){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath, par_bAppend),par_bAppend)){
             
             for(int i=0; i<par_nofRows; i++){
                 
                 int rowOffset = i*par_nofCols;
                 for(int j=0;j<par_nofCols; j++){

                     pw.print(par_arrayValues[rowOffset+j]);
                     if (j<par_nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
                 
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    // save java 2D array of integers to a text file/CSV
    public static void saveArrayToCSV(int[][] par_arrayValues, String par_strOutputFilePath){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath))){
             
             int nofRows = par_arrayValues.length;
             
             for(int i=0; i<nofRows; i++){
                 
                 int nofCols = par_arrayValues[i].length;
                 for(int j=0;j<nofCols; j++){

                     pw.print(par_arrayValues[i][j]);
                     if (j<nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    public static void saveArrayToCSV(int[][] par_arrayValues, String par_strOutputFilePath, boolean par_bAppend){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath, par_bAppend), par_bAppend)){
             
             int nofRows = par_arrayValues.length;
             
             for(int i=0; i<nofRows; ++i){
                 
                 int nofCols = par_arrayValues[i].length;
                 for(int j=0;j<nofCols; ++j){

                     pw.print(par_arrayValues[i][j]);
                     if (j<nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }

    // save java 2D array of doubles to a text file/CSV
    public static void saveArrayToCSV(double[][] par_arrayValues, String par_strOutputFilePath){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath))){
             
             int nofRows = par_arrayValues.length;
             
             for(int i=0; i<nofRows; i++){
                 
                 int nofCols = par_arrayValues[i].length;
                 for(int j=0;j<nofCols; j++){

                     pw.print(par_arrayValues[i][j]);
                     if (j<nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    public static void saveArrayToCSV(double[][] par_arrayValues, String par_strOutputFilePath, boolean par_bAppend){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath, par_bAppend),par_bAppend)){
             
             int nofRows = par_arrayValues.length;
             
             for(int i=0; i<nofRows; ++i){
                 
                 int nofCols = par_arrayValues[i].length;
                 for(int j=0;j<nofCols; ++j){
                     
                     pw.print(par_arrayValues[i][j]);
                     if (j<nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    /**
     * @brief save 2D array to data file (byte arrays)
     *        field 1 : no. of columns (int32/4 bytes)
     *        remaining fields saved column wise for easier loading with MATLAB's fread(fid, [C, inf], 'double')', (double/8 bytes)
     * @param par_arrayValues: data to save
     * @param par_strOutputFilePath: target path
     * @param par_bAppend whether to append or not, row/col fields only written when par_bAppend==False, up to the user to control dimensions when appending
     */
    public static void saveArrayToDataFile(double[][] par_arrayValues, String par_strOutputFilePath, boolean par_bAppend){
        
        try (FileOutputStream pw = new FileOutputStream(new File(par_strOutputFilePath), par_bAppend)) {
            
            int nof_rows = par_arrayValues.length;
            int nof_cols = par_arrayValues[0].length;
            
            if (!par_bAppend){
                pw.write(int2ByteArray(nof_cols));
            }
             
            byte [] arr_values_bytes = new byte[nof_rows*nof_cols*8];
            
            int k =0;
            for(int i=0; i<nof_rows; ++i){
                for(int j=0;j<nof_cols; ++j){

                    ByteBuffer.wrap(arr_values_bytes).putDouble(k, par_arrayValues[i][j]);
                    k += 8;
                }
            }
            pw.write(arr_values_bytes);
            pw.close();
         }
         catch (Exception e){
             System.err.println("Error: " + e.getMessage());
         }
    }
    
    public static void saveArrayToDataFile(double[] par_arrayValues, int par_nof_rows, int par_nof_cols, String par_strOutputFilePath, boolean par_bAppend){
        
        try (FileOutputStream pw = new FileOutputStream(new File(par_strOutputFilePath), par_bAppend)) {
            
            
            if (!par_bAppend){
                pw.write(int2ByteArray(par_nof_cols));
            }
            byte [] arr_values_bytes = new byte[par_nof_rows*par_nof_cols*8];
            
            int k = 0;
            int l = 0;
            for(int i=0; i<par_nof_rows; ++i){
                for(int j=0;j<par_nof_cols; ++j){

                    ByteBuffer.wrap(arr_values_bytes).putDouble(k, par_arrayValues[l]);
                    k += 8;
                    ++l;
                }
            }
            pw.write(arr_values_bytes);
            pw.close();
         }
         catch (Exception e){
             System.err.println("Error: " + e.getMessage());
         }
    }
    
    public static void saveArrayToCSV(float[][] par_arrayValues, String par_strOutputFilePath, boolean par_bAppend){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath, par_bAppend),par_bAppend)){
             
             int nofRows = par_arrayValues.length;
             
             for(int i=0; i<nofRows; ++i){
                 
                 int nofCols = par_arrayValues[i].length;
                 for(int j=0;j<nofCols; ++j){
                     
                     pw.print(par_arrayValues[i][j]);
                     if (j<nofCols-1)
                         pw.print(",");
                 }
                 pw.println();
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    // save matrix of doubles to a text file/CSV. Needs debugging. Does not store pixel values in correct intensity range.
    public static void saveComplexArrayToCSV(ComplexArray par_complexArray, String par_strOutputFilePath){
        
         try(PrintWriter pw = new PrintWriter(new FileWriter(par_strOutputFilePath))){
             
             int [] arrDims = par_complexArray.dims();
             int nofRows = arrDims[DIM_INDEX_ROWS];
             int nofCols = arrDims[DIM_INDEX_COLS]*2;//multiply by 2 for complex to distinguish real from imaginary component
             for(int i=0; i<nofRows; i++){
                 
                 int rowOffset = i*nofCols;
                 for(int j=0;j<nofCols; j+=2){

                     pw.print(par_complexArray.values()[rowOffset+j]);
                     //pw.print(' ');
                     double imag = par_complexArray.values()[rowOffset+j+1];
                     if (imag>=0){
                         pw.print('+');
                         pw.print(Math.abs(imag)); // abs() to prevent +-0.0
                     }
                     else{
                         pw.print(imag);
                     }
                     pw.print('i');

                     if (j<nofCols-2)
                         pw.print(",");
                 }
                 pw.println();
             }          
             pw.close();
         }
         catch (Exception e){

             System.err.println("Error: " + e.getMessage());
         }
    }
    
    /**
     * @brief extract extension from filename, return in lower case letters
     * @param par_filename
     * @return extension string
     */
    public static String extension(String par_filename) {
    
        String ext = "";
        int pos_dot = par_filename.lastIndexOf('.');
        if( pos_dot > 0)
            ext = par_filename.substring(pos_dot + 1).toLowerCase();
        return ext;
    }
    
    /**
     * Convert value to byte array using ieee-be/Big-endian ordering
     * @param value
     * @return byte array
     */
    public static byte[] double2ByteArray(double value) {
        
        byte[] bytes = new byte[8];
        ByteBuffer.wrap(bytes).putDouble(value);
        return bytes;
    }
    
    /**
     * Convert value to byte array using ieee-be/Big-endian ordering
     * @param value
     * @return byte array
     */
    public static byte[] int2ByteArray(int value) {
        
        byte[] bytes = new byte[4];
        ByteBuffer.wrap(bytes).putInt(value);
        return bytes;
    }
    
    public static byte[] reverse_byte_array(byte[] bytes){
        
        byte[] bytes_reversed = new byte[bytes.length];
        for(int i=0, j=bytes.length-1; i<bytes.length; ++i, --j)
            bytes_reversed[i] = bytes[j];
        return bytes_reversed;
    }
        
    public static void test_saveArrayToDataFile(){
        
        String s = "t.data";
        int nof_rows = 4;
        int nof_cols = 3;
        double[][] x = new double[nof_rows][nof_cols];
        for(int r=0; r<nof_rows; ++r){
            for(int c=0; c<nof_cols; ++c){
                x[r][c] = r*10+c+1.234;
            }
        }
        saveArrayToDataFile(x, s, false);
        saveArrayToDataFile(x, s, true);
        return;
    }
}
