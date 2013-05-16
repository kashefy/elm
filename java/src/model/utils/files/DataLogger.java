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
public class DataLogger{
    
    private String m_filepath;
    private int m_nof_cols;
    private int m_cache_size;
    
    private int m_sample_count;
    private double [][] m_arr_sample;
    private float [][] m_arr_sample_float;
    private boolean m_do_float;
    private boolean m_as_bin;
    
    public void set_params(String par_filepath,
            int par_nof_cols){
        
        boolean as_bin = "dat".equals(FileIO.extension(par_filepath));
        this.set_params(par_filepath, par_nof_cols, 2000, false, as_bin);
    }
    
    public void set_params(String par_filepath,
            int par_nof_cols, int par_cache_size,
            boolean par_do_float,
            boolean par_as_bin){
        
        m_filepath = par_filepath;
        m_nof_cols = par_nof_cols;
        m_cache_size = par_cache_size;
        m_do_float = par_do_float;
        m_as_bin = par_as_bin;
    }
    
    public void init(){
        
        m_sample_count = 0;
        m_arr_sample = new double[m_cache_size][m_nof_cols];
        if(m_as_bin){
        
            FileIO.saveArrayToDataFile(new double[1], 0, m_nof_cols, m_filepath, false);
        }
        else{
            FileIO.saveArrayToCSV(new double[1], 0, m_nof_cols, m_filepath); // dummy call to create file
        }
    }
    
    public void add_sample(double [] par_sample){
        
        int destination_row = m_sample_count % m_cache_size;
        System.arraycopy(par_sample, 0, m_arr_sample[destination_row], 0, m_nof_cols);
        
        if(++m_sample_count % m_cache_size == 0){

            if(m_do_float){
                
                float [][] arr_sample_float = new float[m_cache_size][m_nof_cols];
                for(int i=0; i<m_cache_size; ++i){
                    for(int j=0; j<m_nof_cols; ++j){
                        arr_sample_float[i][j] = (float)m_arr_sample[i][j];
                    }
                }
                FileIO.saveArrayToCSV(arr_sample_float, m_filepath, true);
            }
            else{
                if(m_as_bin){
                    FileIO.saveArrayToDataFile(m_arr_sample, m_filepath, true);
                }
                else{
                    FileIO.saveArrayToCSV(m_arr_sample, m_filepath, true);
                }
            }
        }
    }
    
    public void flush(){
        
        int nof_rows_left = m_sample_count % m_cache_size;
        for(int i=0; i<nof_rows_left; ++i){
            
            if(m_do_float){
                
                float [] sample_float = new float[m_nof_cols];
                for(int j=0; j<m_nof_cols; ++j){
                    sample_float[j] = (float)m_arr_sample[i][j];
                }
                FileIO.saveArrayToCSV(sample_float, 1, m_nof_cols, m_filepath, true);
            }
            else{
                if(m_as_bin){
                    double [][] tmp = new double[1][];
                    tmp[0] = m_arr_sample[i];
                    FileIO.saveArrayToDataFile(tmp, m_filepath, true);
                }
                else{
                    FileIO.saveArrayToCSV(m_arr_sample[i], 1, m_nof_cols, m_filepath, true);
                }
            }
        }
    }
    
    public DataLogger(){
        
    }
}
