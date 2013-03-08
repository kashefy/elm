/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.evaluation;

/**
 *
 * @author woodstock
 */
import java.util.Random;

public class Histogram {
    
    private int m_nofBins;
    private int [] m_counts;
    private int m_sum;
    
    private Random m_generator;
    
    public void setParams(int par_nofBins){
        
        if(par_nofBins > 0){
            m_nofBins = par_nofBins;
        }
    }
    
    public void init(){
        
        m_counts = new int [ m_nofBins ];
        m_sum = 0;
        m_generator = new Random();
    }
    
    public void increment(int par_index){
        
        m_counts[ par_index ]++;
    }
    
    public double[] normalize(){
        
        double [] normalizedCounts;
        calcSum();
        normalizedCounts = new double [ m_nofBins ];
        if(m_sum != 0){
            for(int i=0; i<m_nofBins; ++i){

                normalizedCounts[i] = (double)(m_counts[i])/(double)(m_sum);
            }
        }
        return normalizedCounts;
    }
    
    public void reset(){
        
        init();
    }
    
    public int[] getCounts(){
        
        int [] countsToExport = new int[ m_nofBins ];
        System.arraycopy(m_counts, 0, countsToExport, 0, m_nofBins);
        return countsToExport;
    }
    
    public int getCount(int par_index){
        
        return m_counts[ par_index ];
    }
    
    public int findMax(){
        
        int maxIndex = 0;
        int maxVal = 0;
        for(int i=1; i<m_nofBins; ++i){
            
            int candidate = m_counts[i];
            if(candidate > maxVal){
                
                maxIndex = i;
                maxVal = candidate;
            }
            else if(candidate == maxVal){
                
                boolean coin = m_generator.nextBoolean();
                maxIndex = (coin)? maxIndex : i;
            }
        }
        return maxIndex;
    }
    
    public int calcSum(){
        
        m_sum = 0;
        for(int i=0; i<m_nofBins; ++i){
            
            m_sum += m_counts[i];
        }
        return m_sum;
    }
    
    public int getNofBins(){
        
        return m_nofBins;
    }
    
    public void testRun(){
        
        m_nofBins = 6;
        java.util.Random generator = new java.util.Random();
        
        this.setParams(m_nofBins);
        this.init();
        
        int nofTrials = 1000000;
        for(int i=0; i<nofTrials; ++i){
            
            int sample = generator.nextInt(m_nofBins);
            //System.out.println(sample);
            this.increment(sample);
        }
        
        int [] c = this.getCounts();
        
        for(int i=0; i<this.getNofBins(); ++i)
            System.out.print(c[i]+" ");
        System.out.println();
        
        double [] p = this.normalize();
        for(int i=0; i<this.getNofBins(); i++)
            System.out.print(p[i]+" ");
        System.out.println();
        
    }
    
    public Histogram(){
        
    }
    
    
}
