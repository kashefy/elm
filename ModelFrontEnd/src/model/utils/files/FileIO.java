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
import javax.imageio.*;
import java.awt.image.*;

import org.shared.array.*;

public class FileIO {
    
    public static String DIR_NAME_TUNE = "tune";
    public static String DIR_NAME_ACTIVITY_LAYER_F = "activity_layerF";
    public static String DIR_NAME_ACTIVITY_LAYER_Z = "activity_layerZ";
    public static String DIR_NAME_ATTENTION = "att";
    public static String DIR_NAME_WATCH = "watch";
    public static String FILTER_RESPONSE = "filterResponse";
    
    public static int DIM_INDEX_ROWS = 0;
    public static int DIM_INDEX_COLS = 1;
    
    // save matrix to image. Needs debugging. Does not store pixel values in correct intensity range.
    public static void saveArrayToImg(float[] par_arrayValues, int par_width, int par_height, String par_strOutputFilePath){
        
         BufferedImage imgToSave = new BufferedImage(par_width, par_height,BufferedImage.TYPE_BYTE_GRAY);
         WritableRaster wras = imgToSave.getRaster();
         
          // Put the pixels on the raster, using values between 0 and 255.
         for(int h=0; h<par_height; h++){
             
             int rowOffset = h*par_width;
            for(int w=0; w<par_width; w++){
                
                //int value = 127+(int)(128*Math.sin(w/32.)*Math.sin(h/32.)); // Weird sin pattern.
                //wras.setSample(w,h,0,value);
                 wras.setSample(w,h,0,par_arrayValues[ rowOffset+w ]); 
            }
        }
        
         File outputfile = new File(par_strOutputFilePath);
         try{
         
             ImageIO.write(imgToSave, "png", outputfile); // Store the image using the PNG format.
                 
         } catch (IOException e) {

             System.err.println("Error: " + e.getMessage());
         }
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
}
