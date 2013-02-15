/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

/**
 *
 * @author woodstock
 */
import model.utils.files.FileIO;
import java.io.File;

public abstract class AbstractAttention {
    
    protected int m_nofSceneRows;
    protected int m_nofSceneCols;
    protected int m_nofWindowRows;
    protected int m_nofWindowCols;
    protected int m_windowRow;
    protected int m_windowCol;
    protected double [] m_arrScene;
    public int m_shiftY;
    public int m_shiftX;
    
    public void setParams(AttentionParams par_params){
        
        m_nofWindowRows = par_params.m_nofWindowRows;
        m_nofWindowCols = par_params.m_nofWindowCols;
    }
    
    public abstract void init();
    
    public abstract void setScene(double[] par_scene);
    
    public int [] getSceneDims(){
        
        int [] dimsToExport = new int [2];
        dimsToExport[ FileIO.DIM_INDEX_ROWS ] = m_nofSceneRows;
        dimsToExport[ FileIO.DIM_INDEX_COLS ] = m_nofSceneCols;
        return dimsToExport;
    }
    
    /**
     * position window
     */
    public abstract void attend(double [] par_inputs);
    
    public abstract double[] getWindow();    
    
    public int [] getWindowDims(){
        
        int [] dimsToExport = new int [2];
        dimsToExport[ FileIO.DIM_INDEX_ROWS ] = m_nofWindowRows;
        dimsToExport[ FileIO.DIM_INDEX_COLS ] = m_nofWindowCols;
        return dimsToExport;
    }
    
    /**
     * get coordinate of upper left window corner
     */
    public int [] getWindowLoc(){
        
        int [] dimsToExport = new int [2];
        dimsToExport[ FileIO.DIM_INDEX_ROWS ] = m_windowRow;
        dimsToExport[ FileIO.DIM_INDEX_COLS ] = m_windowCol;
        return dimsToExport;
    }
    
    public void save(String par_strPath){
        
        File path = new File(par_strPath);
    }
}
