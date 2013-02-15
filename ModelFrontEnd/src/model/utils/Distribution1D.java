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

public class Distribution1D {
    
    private int m_nofSamples;
    private Random m_generator;
    
    double [] m_cumulSum;
    
    public void setParams(int par_size_pdf){
        
        m_nofSamples = par_size_pdf;
    }
    
    public void init(){
        
        m_generator = new Random();
        m_cumulSum = new double [ m_nofSamples ];
    }
    
    public void evalDistr(double [] par_input){
                
        double [] inputVals = new double [ m_nofSamples ];
        System.arraycopy(par_input, 0, inputVals, 0, m_nofSamples);
        RealArray input = new RealArray(inputVals, m_nofSamples);
        input.uMul(1.0/input.aSum());
        m_cumulSum = ModelUtils.cumulativeSum(input.values());
    }
    
    public int [] sample(int par_nofSamples){
        
        int [] samples = new int[ par_nofSamples ];
        
        for(int si=0; si<par_nofSamples; si++){
            
            double r = m_generator.nextDouble();
            int i=0;
            while(m_cumulSum[i]<r){
                i++;
            }
            
            samples[ si ] = i;
        }
        return samples;
    }    
    
    public int sample(){
                    
        // draw row
        double r = m_generator.nextDouble();
        int i=0;
        while(m_cumulSum[i]<r){
            i++;
        }
        return i;
    }
    
    public void test(){
        
        int size_pdf = 5;
        double [] input = new double [ size_pdf ]; 
        
        this.setParams(size_pdf);
        this.init();
        
        for(int r=0; r<size_pdf; r++){
            
            //input[r] = m_generator.nextDouble();
            input[r] = r;
        }
        
        this.evalDistr(input);
        
        int nofSamples = 1000000;
        int [] samples = this.sample(nofSamples);
        
        int [] counts = new int [ size_pdf ];
        
        for(int si=0; si<nofSamples; si++){
            
            counts[samples[si]]++;
        }
        for(int r=0; r<size_pdf; r++){
            
            System.out.print(input[r]);
            if(r<size_pdf-1)
               System.out.print(",");
            else
                System.out.print(";");
        }
        System.out.println();
        System.out.println();
        
        for(int r=0; r<size_pdf; r++){
            
            System.out.print(counts[r]);
            if(r<size_pdf-1)
               System.out.print(",");
            else
                System.out.print(";");
        }
        System.out.println();
    }
    
    public Distribution1D(){
        
    }
}
