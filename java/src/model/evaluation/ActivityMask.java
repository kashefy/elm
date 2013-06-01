/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.evaluation;

import java.io.File;
import model.utils.files.FileIO;

/**
 *
 * @author woodstock
 */
public class ActivityMask {
    
    private int m_nofNodes;
    private int m_nof_rows;
    private int m_nof_cols;
    private double [] m_arrSum;
    private double [] m_arrSumOfDiff;
    private double [] m_arrActivity;
    private double [] m_prevSample;
    private int m_sampleCount;
    private double m_lowerThresholdExcl; // [0,value)
    private boolean m_bLog;
    private String m_strLogDir;
    private String m_strLogPrefix;
    
    private static String LOG_EXT = ".csv";
    
    public void setParams(int par_nofNodes){
        
        m_nofNodes = par_nofNodes;
    }
    
    public void setParams(int par_nof_rows, int par_nof_cols){
        
        m_nof_rows = par_nof_rows;
        m_nof_cols = par_nof_cols;
        m_nofNodes = m_nof_rows*m_nof_cols;
    }
    
    public void init(){
        
        m_sampleCount = 0;
        m_arrSum = new double [ m_nofNodes ];
        m_arrSumOfDiff = new double [ m_nofNodes ];
        m_arrActivity = new double [ m_nofNodes ];
        m_prevSample = new double [ m_nofNodes ];
        
        if(m_bLog){
            
            FileIO.saveArrayToCSV(m_prevSample, 0, 0, new File(m_strLogDir, m_strLogPrefix + LOG_EXT).getPath(), false);
        }
    }
    
    public void addSample(double [] par_sample){
        
        if(m_sampleCount > 0){
            
            for(int i=0; i<m_nofNodes; ++i){

                m_arrSumOfDiff[i] += Math.abs(m_prevSample[i]-par_sample[i]);
                m_arrSum[i] += Math.abs(par_sample[i]);
            }
        }
        System.arraycopy(par_sample, 0, m_prevSample, 0, m_nofNodes);
        
        if(m_bLog){
            
            FileIO.saveArrayToCSV(par_sample, 1, m_nofNodes, new File(m_strLogDir, m_strLogPrefix + LOG_EXT).getPath(), true);
        }
        m_sampleCount++;
    }
    
    /**
     * operation with previous sample excluded
     * @brief add sub window
     */    
    public void addSample(double [] par_sample, int[] par_loc, int[] par_dims){

        if(m_nof_rows < 0 && m_nof_cols < 0){
            
            System.err.println("Scene dimension not set with set_params(int, int)");
            return;
        }
        int row_start = par_loc[ FileIO.DIM_INDEX_ROWS ];
        int col_start = par_loc[ FileIO.DIM_INDEX_COLS ];
        
        int nof_rows_sub = par_dims[ FileIO.DIM_INDEX_ROWS ];
        int nof_cols_sub = par_dims[ FileIO.DIM_INDEX_COLS ];
        
        if(m_sampleCount > 0){
            
            for(int row_sub=0; row_sub<nof_rows_sub; row_sub++){
                
                int row_offset = (row_start+row_sub)*m_nof_cols;
                int row_offset_sub = row_sub*nof_cols_sub;
                for(int col_sub=0; col_sub<nof_cols_sub; col_sub++){
                    
                    int i = row_offset+col_start+col_sub;
                    m_arrSum[i] += Math.abs(par_sample[row_offset_sub+col_sub]);
                    
                }
            }
        }
        
        if(m_bLog){
            
            FileIO.saveArrayToCSV(par_sample, 1, nof_rows_sub*nof_cols_sub, new File(m_strLogDir, m_strLogPrefix + LOG_EXT).getPath(), true);
        }
        m_sampleCount++;
    }
    
    public double [] calc_activity_diff(){
        
        double [] arrActivityToExp = new double[ m_nofNodes ];
        if(m_sampleCount > 1){
 
            for(int i=0; i<m_nofNodes; i++){

                m_arrActivity[i] = m_arrSumOfDiff[i]/(double)m_sampleCount;
            }
            System.arraycopy(m_arrActivity, 0, arrActivityToExp, 0, m_nofNodes);
        }
        
        return arrActivityToExp;
    }
    
    public double [] calc_activity_intensity(){
        
        double [] arrActivityToExp = new double[ m_nofNodes ];
        if(m_sampleCount > 1){
 
            for(int i=0; i<m_nofNodes; i++){

                m_arrActivity[i] = m_arrSum[i]/(double)m_sampleCount;
            }
            System.arraycopy(m_arrActivity, 0, arrActivityToExp, 0, m_nofNodes);
        }
        
        return arrActivityToExp;
    }
    
    public double [] get_activity(){
        
        double [] arrActivityToExp = new double[ m_nofNodes ];
        System.arraycopy(m_arrActivity, 0, arrActivityToExp, 0, m_nofNodes);
        return arrActivityToExp;
    }
    
    public void set_lower_threshold_excl(double par_lowerThresholdExcl){
        
        m_lowerThresholdExcl = par_lowerThresholdExcl;
    }
    
    /**
     * 
     * @return activity > predefined threshold
     */
    public double [] get_mask(){
        
        double [] arrActivityToExp = new double[ m_nofNodes ];
        for(int i=0; i<m_nofNodes; i++){

            arrActivityToExp[i] = (m_arrActivity[i] > m_lowerThresholdExcl)? 1.0 : 0.0;
        } 
        return arrActivityToExp;
    }
    
    /**
     * 
     * @return activity > provided threshold
     */
    public double [] getMask(double par_lowerThresholdExcl){
        
        set_lower_threshold_excl(par_lowerThresholdExcl);
        double [] mask = get_mask();
        return mask;
    }

    public int get_nof_nodes() {
        return m_nofNodes;
    }

    public void set_nof_nodes(int par_nofNodes) {
        this.m_nofNodes = par_nofNodes;
    }
    
    public void enable_logging(String par_strLogDir, String par_strLogPrefix){
        
        m_bLog = true;
        m_strLogDir     = par_strLogDir;
        m_strLogPrefix  = par_strLogPrefix;
    }
    
    public ActivityMask(){
    
        m_bLog = false;
        m_nof_rows = -1;
        m_nof_cols = -1;
    }
}
