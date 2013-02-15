/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

/**
 *
 * @author woodstock
 */
import model.utils.files.*;

// imports for test_attentionWindow_filterResponse_MNIST()
import model.features.*;
import model.utils.ModelUtils;
import org.shared.array.*;

import java.io.File;

public class AttentionTest {
    
    public static void test_attentionSalient(){
        
        String strMainInputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\input\\MNIST\\";
        File mainInputDir = new File(strMainInputDir);
        String strMainOutputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\MNIST\\";
        File mainOutputDir = new File(strMainOutputDir);
        String strAttOutputDir = "att\\";
        File attOutputDir = new File(mainOutputDir, strAttOutputDir);
        
        DataLoaderImageSetCSV dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strMainInputDir);
        dataLoader.init();
        dataLoader.load();
        double[] stimulus = dataLoader.getSample(25);
       
        int [] stimulusDims = dataLoader.getDims();
        
        AttentionParams attentionParams = new AttentionParams();
        attentionParams.load(new File(mainInputDir, "attentionParamFile.txt").getPath());
        
        AbstractAttention attention = new AttentionSalient();
        attention.setParams(attentionParams);
        attention.init();
        
        attention.setScene(stimulus);
        
        int nofAttentions = 200000;
        int nofAttention_windows_2_display = 64;
        
        int [][] loc_attended_count = new int [ attention.m_nofSceneRows ] [ attention.m_nofSceneCols ];
        int [][] loc_attended_count_centre = new int [ attention.m_nofSceneRows ] [ attention.m_nofSceneCols ];
        
        int [] windowDims = attention.getWindowDims();
        
        for(int ai=0; ai<nofAttentions; ai++){
        
            attention.attend(null);
            int [] windowLoc = attention.getWindowLoc();
            loc_attended_count[ windowLoc[FileIO.DIM_INDEX_ROWS ] ][ windowLoc[FileIO.DIM_INDEX_COLS] ]++;
            
            int [] windowLoc_centre = attention.getWindowLoc();
            windowLoc_centre[FileIO.DIM_INDEX_ROWS ] += windowDims[ FileIO.DIM_INDEX_ROWS ]/2;
            windowLoc_centre[FileIO.DIM_INDEX_COLS ] += windowDims[ FileIO.DIM_INDEX_COLS ]/2;
            
            loc_attended_count_centre[ windowLoc_centre[FileIO.DIM_INDEX_ROWS ] ][ windowLoc_centre[FileIO.DIM_INDEX_COLS] ]++;
            
            double[] window = attention.getWindow();
            if( ai < nofAttention_windows_2_display ){
                
                FileIO.saveArrayToCSV(window, windowDims[ FileIO.DIM_INDEX_ROWS ], windowDims[ FileIO.DIM_INDEX_COLS ], new File(attOutputDir, "window_"+Integer.toString(ai)+".csv").getPath());
            }
        }
        
        FileIO.saveArrayToCSV(loc_attended_count, new File(attOutputDir, "loc_attended_count.csv").getPath());
        FileIO.saveArrayToCSV(loc_attended_count_centre, new File(attOutputDir, "loc_attended_count_centre.csv").getPath());
    }
    
    public static void test_attentionWindow_filterResponse_MNIST(){
        
        String strMainInputDataDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\input\\MNIST\\";
        File mainInputDir = new File(strMainInputDataDir);
        File featParamFile = new File(mainInputDir, "featParamFile_layerF.txt");
      
        String strMainOnputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\MNIST\\";
        File mainOutputDir = new File(strMainOnputDir);
        File attOutputDir = new File(mainOutputDir, "att");
        File filterOutputDir = new File(attOutputDir, "filterResponse");
       
        OrientationMap mapOrient = new OrientationMap();

        FeatureMapParams featParams = new FeatureMapParams();
        featParams.load(featParamFile.getPath());

        mapOrient.setStartingScale(0.2*16/featParams.getSupportRadius());
        mapOrient.setParams(featParams);
        //mapOrient.setStartingScale(0.15*featParams.getSupportRadius()/16);
        mapOrient.init();
        mapOrient.saveFilters(filterOutputDir.getPath());

        AbstractDataLoader dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(mainInputDir.getPath());
        dataLoader.init();
        dataLoader.load();
        double [] stimulus = dataLoader.getSample(0);
        //       for(int i=0; i<nofRows * nofCols; i++){
        //            
        //            stimulusRealVals[i] = 1-stimulusRealVals[i];
        //        }
        
        AttentionParams attentionParams = new AttentionParams();
        attentionParams.load(new File(mainInputDir, "attentionParamFile.txt").getPath());
        
        AbstractAttention attention = new AttentionSalient();
        attention.setParams(attentionParams);
        attention.init();
        attention.setScene(stimulus);
        attention.attend(null);
        double[] windowRealVals = attention.getWindow();
        int [] windowDims = attention.getWindowDims();
        double [] windowComplexVals = ModelUtils.prepRealValuesForComplex(windowRealVals);
        ComplexArray windowComplex = new ComplexArray(windowComplexVals, windowDims[ FileIO.DIM_INDEX_ROWS ], windowDims[ FileIO.DIM_INDEX_COLS ], 2);

        mapOrient.setStimulus(windowComplex);
        ComplexArray[] output = mapOrient.convolve();
        RealArray[] outputRect = OrientationMapTest.rectify(output);

        String strExt = ".csv";

        FileIO.saveArrayToCSV(windowRealVals, windowDims[ FileIO.DIM_INDEX_ROWS ], windowDims[ FileIO.DIM_INDEX_COLS ], new File(filterOutputDir, "stimRe" + strExt).getPath());

        for(int i=0; i<output.length; i++){

           String strPrefix;

           strPrefix = Integer.toString(i);
           FileIO.saveComplexArrayToCSV(output[i],new File(filterOutputDir, strPrefix + "out" + strExt).getPath());
           int [] nDims = output[i].torRe().dims(); 
           //FileIO.saveArrayToCSV(output[i].torRe().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],new File(filterOutputDir, strPrefix + "outRe" + strExt).getPath());
           //FileIO.saveArrayToCSV(output[i].torIm().values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],new File(filterOutputDir, strPrefix + "outIm" + strExt).getPath());
           FileIO.saveArrayToCSV(outputRect[i].values(),nDims[FileIO.DIM_INDEX_ROWS],nDims[FileIO.DIM_INDEX_COLS],new File(filterOutputDir, strPrefix + "outAbs" + strExt).getPath());
        }
    }
}
