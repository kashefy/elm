/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

import java.util.Random;

/**
 *
 * @author woodstock
 */
public class AttentionUniform extends AbstractAttention{
    
    Random m_generator;
    
    @Override
    public void init(){
       
        m_generator = new Random();
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
        m_windowRow = m_generator.nextInt(m_nofSceneRows-m_nofWindowRows+1);
        m_windowCol = m_generator.nextInt(m_nofSceneCols-m_nofWindowCols+1);
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
   
    public AttentionUniform(){
        
        m_nofSceneRows = 28;
        m_nofSceneCols = 28;
        m_nofWindowRows = 20;
        m_nofWindowCols = 20;
        m_windowRow = 0;
        m_windowCol = 0;
    }
}