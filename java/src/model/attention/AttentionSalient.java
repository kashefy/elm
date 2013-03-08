/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

import java.util.Random;
import model.utils.files.FileIO;

/**
 *
 * @author woodstock
 */
public class AttentionSalient extends AbstractAttention{
    
    Saliency m_saliency;
    
    @Override
    public void setParams(AttentionParams par_params){
        
        super.setParams(par_params);
        m_saliency.setParams(par_params.get_saliencyParamsRef());
        m_nofSceneRows = par_params.get_saliencyParamsRef().m_nofRows;
        m_nofSceneCols = par_params.get_saliencyParamsRef().m_nofCols;
    }
    
    @Override
    public void init(){
       
        m_saliency.init();
    }
    
    @Override
    public void setScene(double[] par_scene){
        
        int nofScenePixels = par_scene.length;
        m_arrScene = new double[ nofScenePixels ];
        System.arraycopy(par_scene, 0, m_arrScene, 0, nofScenePixels);
        m_saliency.eval(m_arrScene);
    }
    
    /**
     * Attend to next window in scene.
     * @param par_inputs pass null
     */
    @Override
    public void attend(double [] par_inputs){
        
        int [] winCentreRef = new int[2];
        m_saliency.measure(winCentreRef);
        
        m_windowRow = winCentreRef[ FileIO.DIM_INDEX_ROWS ];
        m_windowRow = rectifyWindowCoordinate(m_windowRow, m_nofWindowRows, m_nofSceneRows);
        
        m_windowCol = winCentreRef[ FileIO.DIM_INDEX_COLS ];
        m_windowCol = rectifyWindowCoordinate(m_windowCol, m_nofWindowCols, m_nofSceneCols);
        
//        System.out.print(m_windowRow);
//        System.out.print(":");
//        System.out.print(m_windowCol);
//        System.out.println();
    }
    
    /**
     * Adjust window coordinate so that window is within scene dimension
     * @param par_windowCentreCoord window centre coordinate to rectify
     * @param par_windowDim window dimension
     * @param par_sceneDim scene dimension
     * @return adjusted window coordinate
     */
    private int rectifyWindowCoordinate(int par_windowCentreCoord, int par_windowDim, int par_sceneDim){
        
        int windowRadius = (int) Math.ceil(par_windowDim/2.0);
        int windowCoord = par_windowCentreCoord - windowRadius;
        if (windowCoord < 0)
            windowCoord = 0;
        else if(windowCoord + par_windowDim - 1 >= par_sceneDim)
            windowCoord = par_sceneDim - par_windowDim - 1;
        return windowCoord;
    }
    
    @Override
    public double[] getWindow(){
        
        double [] window = new double [ m_nofWindowRows*m_nofWindowCols ]; 
        
        int rStart = m_windowRow;
        int rEnd = rStart + m_nofWindowRows;
        int cStart = m_windowCol;
        for(int r=rStart; r<rEnd; ++r){

            int rWindow = r-rStart;
                        
//            System.out.print(r);
//            System.out.print(" ");
//            System.out.print(rWindow);
//            System.out.println();
            
            System.arraycopy(m_arrScene, r*m_nofSceneCols+cStart, window, rWindow*m_nofWindowCols, m_nofWindowCols);
        }
        return window;
    }
    
    @Override
    public void save(String par_strPath){
        
        m_saliency.save(par_strPath);
    }
   
    public AttentionSalient(){
        
        m_nofSceneRows = -1;
        m_nofSceneCols = -1;
        
        m_saliency = new Saliency();
    }
}