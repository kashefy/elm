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
import org.shared.array.RealArray;
import model.utils.files.FileIO;

public class Distribution2D {
    
    private int m_nofSamples;
    private int m_nofRows;
    private int m_nofCols;
    private Random m_generator;
    int [] m_arrTransposePermutation;
    
    double [][] m_arrCumulSumPerCol;
    double [] m_cumulSumOfCumulSum;
    
    public void setParams(int par_nofRows, int par_nofCols){
        
        m_nofRows = par_nofRows;
        m_nofCols = par_nofCols;
    }
    
    public void init(){
        
        m_nofSamples = m_nofRows * m_nofCols;
        m_generator = new Random();
        m_arrTransposePermutation = new int[]{1,0}; // {1,0} for traditional, 2D transposition of matrices use the permutation (dim 1 goes to dim 0, and vice versa).
        m_arrCumulSumPerCol = new double [ m_nofCols ][ m_nofRows ];
    }
    
    public void evalDistr(double [] par_input){
        
        double [] inputVals = new double [ m_nofSamples ];
        System.arraycopy(par_input, 0, inputVals, 0, m_nofSamples);
        RealArray input = new RealArray(inputVals, m_nofRows, m_nofCols);
        
        
        //System.out.printf("i =%n%s%n", input.toString());
        //input.uAdd(-input.aMean());
        //input.uExp();
        //System.out.printf("exp(i) =%n%s%n", input.toString());
        
        RealArray inputT = input.transpose(m_arrTransposePermutation);
        
        // calc accumulative sum for orginial columns
        RealArray temp;
        double [] arrFinalElements = new double [ m_nofCols ];
        int lastIndex = m_nofRows-1;
        for(int i=0, c=0; i<m_nofCols; i++, c+=m_nofRows){
            
            double [] colVals = new double[ m_nofRows ];
            System.arraycopy(inputT.values(), c, colVals, 0, m_nofRows);
            
            double [] cumulSumPerCol = ModelUtils.cumulativeSum(colVals);
            arrFinalElements[i] = cumulSumPerCol[ lastIndex ];
            
            temp = new RealArray(colVals);
            double localSum = temp.aSum();
            temp.uMul(1.0/localSum);
            m_arrCumulSumPerCol[i] = ModelUtils.cumulativeSum(temp.values());
                    
            //System.out.printf("c =%n%s%n", temp.toString());
        }
        temp = new RealArray(arrFinalElements);
        double localSum = temp.aSum();
        temp.uMul(1.0/localSum);
        
        m_cumulSumOfCumulSum = ModelUtils.cumulativeSum(temp.values());
 
        //System.out.printf("i =%n%s%n", input.toString());
        //System.out.printf("iT =%n%s%n", inputT.toString());
        
//        System.out.println();
//        for(int c=0; c<m_nofCols; c++){
//            
//            for(int r=0; r<m_nofRows; r++){
//                
//                System.out.print(m_arrCumulSumPerCol[c][r]+" ");
//            }
//            System.out.println();
//        }
        
        //temp = new RealArray(m_cumulSumOfCumulSum);
        //System.out.printf("c =%n%s%n", temp.toString());
    }
    
    public int[][] sample(int par_nofSamples){
        
        int [][] samples = new int[ par_nofSamples ][2];
        
        for(int si=0; si<par_nofSamples; si++){
            
            double c = m_generator.nextDouble();
            int j=0;
            while(m_cumulSumOfCumulSum[j]<c){
                j++;
            }
            
            double r = m_generator.nextDouble();
            int i=0;
            while(m_arrCumulSumPerCol[j][i]<r){
                i++;
            }
            
            samples[ si ][ FileIO.DIM_INDEX_ROWS ] = i;
            samples[ si ][ FileIO.DIM_INDEX_COLS ] = j;
        }
        
        return samples;
    }    
    
    public int sample(){
                    
        // draw column
        double c = m_generator.nextDouble();
        int j=0;
        while(m_cumulSumOfCumulSum[j]<c){
            j++;
        }

        // draw row
        double r = m_generator.nextDouble();
        int i=0;
        while(m_arrCumulSumPerCol[j][i]<r){
            i++;
        }
        
        return i*m_nofCols + j;
        
    }
    
    public void test(){
        
        int nofRows = 3;
        int nofCols = 4;
        
        double [] input = new double [ nofRows*nofCols ]; 
        
        this.setParams(nofRows, nofCols);
        this.init();
        
        int i=0;
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                //input[i++] = m_generator.nextGaussian();
                input[i++] = r+c;
            }
        }
        
        this.evalDistr(input);
        
        int nofSamples = 1000000;
        int [][] samples = this.sample(nofSamples);
        
        int [][] counts = new int [ nofRows ][ nofCols ];
        
        for(int si=0; si<nofSamples; si++){
            
            counts[samples[si][0]][samples[si][1]]++;
            
        }
        
        
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                System.out.print(input[r*nofCols+c]);
                if(c<nofCols-1)
                   System.out.print(",");
                else
                    System.out.print(";");
            }
        }
        System.out.println();
        System.out.println();
        
        for(int r=0; r<nofRows; r++){
            
            for(int c=0; c<nofCols; c++){
                
                System.out.print(counts[r][c]);
                if(c<nofCols-1)
                   System.out.print(",");
                else
                    System.out.print(";");
            }
        }
        System.out.println();
        
        
    }
    
    public Distribution2D(){
        
    }
}
