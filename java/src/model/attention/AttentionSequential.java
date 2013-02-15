/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

import java.io.File;
import model.utils.files.FileIO;

/**
 *
 * @author woodstock
 */
public class AttentionSequential extends AbstractAttention{
    
    @Override
    public void init(){
       
        
    }
    
    @Override
    public void setScene(double[] par_scene){
        
        int nofScenePixels = par_scene.length;
        m_arrScene = new double[ nofScenePixels ];
        System.arraycopy(par_scene, 0, m_arrScene, 0, nofScenePixels);
    }
    
    @Override
    public void attend(double [] par_inputs){
        
        //m_windowCol++;
        m_windowCol += m_shiftX;
        if(m_windowCol == m_nofSceneCols-m_nofWindowCols+1){
            
            m_windowCol = 0;
            //m_windowRow++;
            m_windowRow += m_shiftY;
            if(m_windowRow == m_nofSceneRows-m_nofWindowRows+1){
                
                // time to stop
            }
        }
    }
    
    @Override
    public double[] getWindow(){
        
        double [] window = new double [ m_nofWindowRows*m_nofWindowCols ]; 
        
        int rStart = m_windowRow;
        int rEnd = rStart + m_nofWindowRows;
        int cStart = m_windowCol;
        for(int r=rStart; r<rEnd; r++){
            
            int rWindow = r-rStart;
            System.arraycopy(m_arrScene, r*m_nofSceneCols+cStart, window, rWindow*m_nofWindowCols, m_nofWindowCols);
        }
        return window;
    } 
   
    
    public AttentionSequential(){
        
        m_nofSceneRows = 28+28;
        m_nofSceneCols = 28+28;
        m_nofWindowRows = 28;
        m_nofWindowCols = 28;
        m_windowRow = 0;
        m_windowCol = 0;
        m_shiftY = 1;
        m_shiftX = 1;
    }
}
