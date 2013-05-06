/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils.files;

import java.io.File;

/**
 *
 * @author woodstock
 */
public class DataLoggerInt {
    
    private String m_filepath;
    private int m_nof_cols;
    private int m_cache_size;
    
    private int m_sample_count;
    private int [][] m_arr_sample;
    
    public void set_params(String par_filepath,
            int par_nof_cols, int par_cache_size){
        
        m_filepath = par_filepath;
        m_nof_cols = par_nof_cols;
        m_cache_size = par_cache_size;
    }
    
    public void init(){
        
        m_sample_count = 0;
        m_arr_sample = new int[m_cache_size][m_nof_cols];
        FileIO.saveArrayToCSV(new double[1], 0, m_nof_cols, m_filepath); // dummy call to create file
    }
    
    public void add_sample(int [] par_sample){
        
        int destination_row = m_sample_count % m_cache_size;
        System.arraycopy(par_sample, 0, m_arr_sample[destination_row], 0, m_nof_cols);
        if(++m_sample_count % m_cache_size == 0){
            
            FileIO.saveArrayToCSV(m_arr_sample, m_filepath, true);
        }
    }
    
    public void flush(){
        
        int nof_rows_left = m_sample_count % m_cache_size;
        for(int i=0; i<nof_rows_left; ++i){
            
            FileIO.saveArrayToCSV(m_arr_sample[i], 1, m_nof_cols, m_filepath, true);
        }
        
    }
    
    public DataLoggerInt(){
        
    }
}
