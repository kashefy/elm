/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils;

/**
 *
 * @author woodstock
 */
import java.util.Arrays;
import java.util.Random;
import model.utils.files.FileIO;
import org.shared.array.*;

public class ModelUtils {
    
    public static int MODE_PAD_ROWS_BOTTOM = 10;
    public static int MODE_PAD_ROWS_TOP = 11;
    public static int MODE_PAD_COLS_RIGHT = 20;
    public static int MODE_PAD_COLS_LEFT = 21;
    public static int MODE_PAD_BOTH_SURROUND = 30;
    public static int MODE_PAD_BOTH_BOTTOMRIGHT = 31;
    
    // using weighted average performed by Matlab's rgb2gray(): 0.2989 * R + 0.5870 * G + 0.1140 * B
    public static float RGB2gray(int red, int green, int blue){
        
        float intensity;
        intensity = (float)0.2989 * red + (float)0.5870 * green + (float)0.1140 * blue;
        return intensity;
    }   
    
    public static ComplexArray realArrayToComplex(RealArray par_realArray){
        
       int nofDims = par_realArray.dims().length;
       int [] arrDims = new int [ nofDims + 1];
       for(int i=0; i<nofDims; ++i){
           arrDims[i] = par_realArray.dims()[i];
       }
       arrDims[ nofDims ] = 2;
       
       double [] complexVals = ModelUtils.prepRealValuesForComplex(par_realArray.values());
       ComplexArray complexArray = new ComplexArray(complexVals,arrDims);
       
       //System.out.printf("ri =%n%s%n", par_realArray.toString());
       //System.out.printf("ci =%n%s%n", complexArray.toString());
       
       return complexArray;
    }
    
    public static double [] prepRealValuesForComplex(double [] par_realVals){
        
        int length = par_realVals.length;
        double [] par_complexVals = new double[ length*2 ];
        
        int i2;
        for(int i=0; i<length; ++i){
               
            i2 = i*2;
            par_complexVals [  i2  ] = par_realVals[ i ];
            par_complexVals [ i2+1 ] = 0; // zero for imaginary part
        }
        
        return par_complexVals;
    }
    
    // will pad a complex 2D array to the given dimensions
    // if given dimensions are smaller than input dimensions, equivalent to truncation
    public static ComplexArray pad2D(ComplexArray par_input, int [] par_newDims){
        
        ComplexArray padded;
        int [] oldDims = par_input.dims();
        int nofOldRows = oldDims[FileIO.DIM_INDEX_ROWS];
        int nofOldCols = oldDims[FileIO.DIM_INDEX_COLS];
        padded = new ComplexArray(par_newDims);
        
        int nofRowsToOp = Math.min(nofOldRows, par_newDims[0]);
        int nofColsToOp = Math.min(nofOldCols, par_newDims[1]);
        
        // we can assume that elements beyond the old dimensions are already zero
        // we only need to insert the elements from the old matrix
        for(int r=0; r<nofRowsToOp; ++r){

            for(int c=0; c<nofColsToOp; ++c){

                padded.set(par_input.get(r, c, 0), r, c, 0);// real component
                padded.set(par_input.get(r, c, 1), r, c, 1);// imaginary component
            }
        }

        //System.out.printf("i =%n%s%n", new ComplexArray(par_input.values(),par_input.dims())); 
        //System.out.printf("p =%n%s%n", padded.toString()); 
        
        return padded;
    }
    
    // will pad a real valued 2D array to the given dimensions
    // if given dimensions are smaller than input dimensions, equivalent to truncation
    public static RealArray pad2D(RealArray par_input, int [] par_newDims){
        
        RealArray padded;
        int [] oldDims = par_input.dims();
        int nofOldRows = oldDims[0];
        int nofOldCols = oldDims[1];
        padded = new RealArray(par_newDims);
        
        int nofRowsToOp = Math.min(nofOldRows, par_newDims[0]);
        int nofColsToOp = Math.min(nofOldCols, par_newDims[1]);
            
        // we can assume that elements beyond the old dimensions are already zero
        // we only need to insert the elements from the old matrix
        for(int r=0; r<nofRowsToOp; ++r){

            for(int c=0; c<nofColsToOp; ++c){

                padded.set(par_input.get(r, c), r, c);
            }
        }
        
        return padded;
    }
    
    public static RealArray pad2D(RealArray par_input, int par_bandWdith, int par_nMode){
        
        RealArray padded;
        int [] oldDims = par_input.dims();
        int nofRowsOld = oldDims[ FileIO.DIM_INDEX_ROWS ];
        int nofColsOld = oldDims[ FileIO.DIM_INDEX_COLS ];
        
        int [] newDims = new int[2];
        int nofRowsNew = nofRowsOld;
        int nofColsNew = nofColsOld;
        
        if(par_nMode == MODE_PAD_BOTH_BOTTOMRIGHT){
            
            nofRowsNew += par_bandWdith;
            nofColsNew += par_bandWdith;
        }
        else if(par_nMode == MODE_PAD_BOTH_SURROUND){
            
            nofRowsNew += par_bandWdith + par_bandWdith;
            nofColsNew += par_bandWdith + par_bandWdith;
        }
        else if(par_nMode == MODE_PAD_COLS_LEFT){
         
            nofColsNew += par_bandWdith;
        }
        else if(par_nMode == MODE_PAD_COLS_RIGHT){
            
            nofColsNew += par_bandWdith;
        }
        else if(par_nMode == MODE_PAD_ROWS_BOTTOM){
         
            nofRowsNew += par_bandWdith;
        }
        else if(par_nMode == MODE_PAD_ROWS_TOP){
            
            nofRowsNew += par_bandWdith;
        }
        
        newDims[ FileIO.DIM_INDEX_ROWS ] = nofRowsNew;
        newDims[ FileIO.DIM_INDEX_COLS ] = nofColsNew;
        padded = new RealArray(newDims);
        
        int nofRowsToOp = Math.min(nofRowsOld, nofRowsNew);
        int nofColsToOp = Math.min(nofColsOld, nofColsNew);
        
        int rowStart = 0;
        int colStart = 0; 
           
        if(par_nMode == MODE_PAD_BOTH_BOTTOMRIGHT){
            
            rowStart = 0;
            colStart = 0;
        }
        else if(par_nMode == MODE_PAD_BOTH_SURROUND){
            
            rowStart = par_bandWdith;
            colStart = par_bandWdith;
        }
        else if(par_nMode == MODE_PAD_COLS_LEFT){
        
            rowStart = 0;
            colStart = par_bandWdith;
        }
        else if(par_nMode == MODE_PAD_COLS_RIGHT){
            
            rowStart = 0;
            colStart = 0;
        }
        else if(par_nMode == MODE_PAD_ROWS_BOTTOM){
         
            rowStart = 0;
            colStart = 0;  
        }
        else if(par_nMode == MODE_PAD_ROWS_TOP){
            
            rowStart = par_bandWdith;
            colStart = 0;
        }
        // we can assume that elements beyond the old dimensions are already zero
        // we only need to insert the elements from the old matrix
        int r = rowStart;
        for(int ri=0; ri<nofRowsToOp; ++ri){

            int c = colStart;
            for(int ci=0; ci<nofColsToOp; ++ci){

                padded.set(par_input.get(ri, ci), r, c);
                ++c;
                
                //System.out.printf("y =%n%s%n", padded.toString());
            }
            ++r;
        }
        
        return padded;
    }
            
    public static int [][] pad2D(int par_arrInput[][], int par_bandWdith, int par_nMode){
        
        int [][] arrOutput;
        int nofRows = par_arrInput.length;
        int nofCols = (nofRows > 0)? par_arrInput[0].length : 0;
            
        if(par_nMode == MODE_PAD_COLS_RIGHT){
            
            arrOutput = new int[ nofRows ][ nofCols+par_bandWdith ];
        }
        else if(par_nMode == MODE_PAD_ROWS_BOTTOM){
            
            arrOutput = new int[ nofRows+par_bandWdith ][ nofCols ];
        }
        else{ // MODE_PAD_BOTH_BOTTOMRIGHT
            arrOutput = new int[ nofRows+par_bandWdith ][ nofCols+par_bandWdith ];
        }
       
        for(int row=0; row<nofRows; row++){
            
            System.arraycopy(par_arrInput[ row ], 0, arrOutput[ row ], 0, nofCols);
        }
        return arrOutput;
    }
    
    public static int [] extractColumns(int par_arrInput[][], int par_columnOfInterest){
        
        int nofRows = par_arrInput.length;
        int [] column = new int[ nofRows ];
        for(int row=0; row<nofRows; ++row){
            
            column[ row ] = par_arrInput[ row ][ par_columnOfInterest ];
        }
        return column;
    }
    
    public static int [][] extractColumns(int par_arrInput[][], int par_columnRangeOfInterestStart, int par_columnRangeOfInterestSEnd){
        
        // start and end columns included
        int nofRows = par_arrInput.length;
        int nofCols = par_columnRangeOfInterestSEnd-par_columnRangeOfInterestStart+1;
        int [][] columnsOfInterest = new int[ nofRows ][ nofCols ];
        for(int row=0; row<nofRows; ++row){
            
            for(int col=par_columnRangeOfInterestStart; col<=par_columnRangeOfInterestSEnd; ++col){
               
                columnsOfInterest[ row ][ col ] = par_arrInput[ row ][ col ];
            }
        }
        return columnsOfInterest;
    }
    
    public static boolean insertColumn(int par_arr_new_column[], int par_arr_dst[][], int par_pos){
        
        int nofRows = par_arr_dst.length;
        int size_new_col = par_arr_new_column.length;
        if(size_new_col != nofRows)
            return false;
        else{
         
            for(int row=0; row<nofRows; ++row){

                par_arr_dst[ row ][ par_pos ] = par_arr_new_column[ row ];
            }
        }
        return true;
    }
    
    public static double [] cumulativeSum(double [] par_arrInput){
        
        int nofInputs = par_arrInput.length;
        double [] cumSum = new double [ nofInputs ];
        double intermSum = 0;
        for(int i=0; i<nofInputs; ++i){
            
            intermSum += par_arrInput[i];
            cumSum[i] = intermSum;
        }
        return cumSum;
    }
    
    /**
     * @brief perform partial cumulative sum, until limit is reached
     * 
     * Used for faster processing
     * 
     * @param par_arrInput
     * @param par_partial_limit
     * @return elements beyond partial limit will be zeros
     */
    public static double [] cumulativeSum(double [] par_arrInput, double par_partial_limit){
        
        if(par_partial_limit < 0){
            
            return ModelUtils.cumulativeSum(par_arrInput); // standard case, limit is 1
        }
        else{
            int nofInputs = par_arrInput.length;
            double [] cumSum = new double [ nofInputs ];
            double intermSum = 0;
            for(int i=0; i<nofInputs && intermSum<par_partial_limit; ++i){

                intermSum += par_arrInput[i];
                cumSum[i] = intermSum;
            }
            return cumSum;
        }
    }
    
    /*
     * local accumulative sum [start, start+n)
     */
    public static double [] cumulativeSum(double [] par_arrInput, int par_start, int par_nofElements){
        
        int nofInputs = par_nofElements;
        double [] cumSum = new double [ nofInputs ];
        double intermSum = 0;
        for(int i=par_start, reli=0; reli<par_nofElements; ++i, ++reli){
            
            intermSum += par_arrInput[i];
            cumSum[ reli ] = intermSum;
        }
        return cumSum;
    }
        
    /**
     * @brief calculate the next power for a given base starting form given input
     * @param par_input
     * @param par_base
     * @return next power
     */  
    public static int nextPow(int par_input, int par_base){
        
        boolean bOriginallyNegative = false;
        if(par_input <0){
            par_input = -par_input;
            bOriginallyNegative = true;
        }
        
        //int pow2 = 1;
        //while (pow2 < par_input){
            
            //pow2 *= par_base;
        //}
        
        double logBase = Math.log(par_base);
        double inputLog = Math.log(par_input);
        int exponent =(int)Math.ceil(inputLog/logBase);
        int nextPow = (int)Math.pow(par_base, (double)exponent);
        if(bOriginallyNegative)
            nextPow = -nextPow;
        
        return nextPow;
    }
        
    public static RealArray calcNeighVar(RealArray par_input, int par_neighborhood){
        
        int [] dims = par_input.dims();
        
        int nofRows = dims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = dims[ FileIO.DIM_INDEX_COLS ];
        
        RealArray neighVar = new RealArray(dims);
        
        for(int r=0; r<nofRows; r++){
            
            int rStartiIncl = r-par_neighborhood;
            if(rStartiIncl < 0){
                rStartiIncl = 0;
            }
            
            int rEndiExcl = r+par_neighborhood+1;
            if(rEndiExcl > nofRows){
                rEndiExcl = nofRows;
            }
            
            for(int c=0; c<nofCols; c++){
                
                int cStartiIncl = c-par_neighborhood;
                if(cStartiIncl < 0){
                    cStartiIncl = 0;
                }
            
                int cEndiExcl = c+par_neighborhood+1;
                if(cEndiExcl > nofCols){
                    cEndiExcl = nofCols;
                }
                RealArray sub = par_input.subarray(rStartiIncl,rEndiExcl,cStartiIncl,cEndiExcl);
                double var = sub.aVar();
                neighVar.set(var, r,c);
            }
        }
        
        return neighVar;
        
    }
    
    public static RealArray normalizeMm(RealArray par_input, int par_neighborhood, double par_M){
        
        int [] dims = par_input.dims();
        
        int nofRows = dims[ FileIO.DIM_INDEX_ROWS ];
        int nofCols = dims[ FileIO.DIM_INDEX_COLS ];
        
        // normalize to [0,M]
        RealArray normM = new RealArray(dims);
        normM = par_input.clone();
        normM = normM.uAdd(-normM.aMin());
        normM = normM.uMul(par_M/normM.aMax());
        
        double sumLocalMax = 0;
        int nofLocalMax = 0;
        boolean bExcludeGlobalMax = true;
        
        for(int r=0; r<nofRows; r++){
            
            int rStartiIncl = r-par_neighborhood;
            if(rStartiIncl < 0){
                rStartiIncl = 0;
            }
            
            int rEndiExcl = r+par_neighborhood+1;
            if(rEndiExcl > nofRows){
                rEndiExcl = nofRows;
            }
            
            for(int c=0; c<nofCols; c++){
                
                int cStartiIncl = c-par_neighborhood;
                if(cStartiIncl < 0){
                    cStartiIncl = 0;
                }
            
                int cEndiExcl = c+par_neighborhood+1;
                if(cEndiExcl > nofCols){
                    cEndiExcl = nofCols;
                }
                RealArray sub = normM.subarray(rStartiIncl,rEndiExcl,cStartiIncl,cEndiExcl);
                double localMax  = sub.aMax();
                if(localMax == par_M && bExcludeGlobalMax){
                    
                    bExcludeGlobalMax = false;
                }
                else{
                    sumLocalMax += localMax;
                    nofLocalMax++;
                }
            }
        }
        double meanLocalMax = sumLocalMax/(double)nofLocalMax;
        double normTerm = par_M-meanLocalMax;
        normTerm *= normTerm;
        
        normM = normM.uMul(normTerm);
        
        return normM;
    }
    
    /**
     * @brief indicies that would sort the array
     * @param par_input_vals
     * @return array of indicies to sort that input array
     */
    public static int[] argsort(int [] par_input_vals){
        
        int n = par_input_vals.length;
        int [] indicies = new int[n];
        int [] input_vals = new int[n];
        System.arraycopy(par_input_vals, 0, input_vals, 0, n);
        Arrays.sort(input_vals);
        // what happens for duplicate values in the array?
        for(int i=0; i<n; ++i){
            indicies[i] = Arrays.binarySearch(input_vals, par_input_vals[i]);
        }
        return indicies;
    }
    
    /**
     * @see ModelUtils#rand(int[])
     */
    public static int[] argsort(double [] par_input_vals){
        
        int n = par_input_vals.length;
        int [] indicies = new int[n];
        double [] input_vals = new double[n];
        System.arraycopy(par_input_vals, 0, input_vals, 0, n);
        Arrays.sort(input_vals);
        // what happens for duplicate values in the array?
        for(int i=0; i<n; ++i){
            indicies[i] = Arrays.binarySearch(input_vals, par_input_vals[i]);
        }
        return indicies;
    }
    
    /**
     * @breif argument of the maximum
     * @param par_input
     * @return argmax
     */
    public static int argmax(double [] par_input){
        
        int arg_max = 0;
        if (par_input != null){
            double max = par_input[0];
            for(int i=1; i<par_input.length; ++i){

                double candidate = par_input[i];
                if(candidate > max){

                    max = candidate;
                    arg_max = i;
                }
            }
        }
        return arg_max;
    }

    /**
     * @breif argument of the maximum
     * @param par_input
     * @return argmax
     */
    public static int argmax(int [] par_input){
        
        int arg_max = 0;
        if (par_input != null){
            int max = par_input[0];
            for(int i=1; i<par_input.length; ++i){

                int candidate = par_input[i];
                if(candidate > max){

                    max = candidate;
                    arg_max = i;
                }
            }
        }
        return arg_max;
    }
    
    /**
     * @brief argument of the minimum
     * @param par_input
     * @return argmin
     */
    public static int argmin(double [] par_input){
        
        int arg_min = 0;
        if (par_input != null){
            double max = par_input[0];
            for(int i=1; i<par_input.length; ++i){

                double candidate = par_input[i];
                if(candidate < max){

                    max = candidate;
                    arg_min = i;
                }
            }
        }
        return arg_min;
    }

    /**
     * @brief argument of the minimum
     * @param par_input
     * @return argmin
     */
    public static int argmin(int[] par_input){
        
        int arg_min = 0;
        if (par_input != null){
            int max = par_input[0];
            for(int i=1; i<par_input.length; ++i){

                int candidate = par_input[i];
                if(candidate < max){

                    max = candidate;
                    arg_min = i;
                }
            }
        }
        return arg_min;
    }

    /**
     * @brief fill array x with random numbers sampled from a uniform distribution
     * @param x array to populate
     */
    public static void rand(int [] x){
        
        Random generator = new Random();
        for(int i=0; i<x.length; ++i){
            
            x[i] = generator.nextInt();
        }
        return;
    }
      
    /**
     * @see ModelUtils#rand(int[])
     */
    public static void rand(double [] x){
        
        Random generator = new Random();
        for(int i=0; i<x.length; ++i){
            
            x[i] = generator.nextInt();
        }
        return;
    }
}
