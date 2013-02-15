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
import java.io.File;

public class SaliencyTest {
    
    public static void testEval(){
        
        String strMainInputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\input\\MNIST\\";
        String strOutputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\repo-git\\trunk\\ModelFrontEnd\\data\\output\\MNIST\\filterResponse\\";
        
        DataLoaderImageSetCSV dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strMainInputDir);
        dataLoader.init();
        dataLoader.load();
        double[] stimulus = dataLoader.getSample(25);
       
        int [] stimulusDims = dataLoader.getDims();
        
        SaliencyParams saliencyParams = new SaliencyParams();
        saliencyParams.load(new File(strMainInputDir, "saliencyParamFile.txt").getPath());
        
        Saliency saliencyMap = new Saliency();
        saliencyMap.setParams(saliencyParams);
        saliencyMap.init();
        
        saliencyMap.eval(stimulus);
        FileIO.saveArrayToCSV(stimulus, stimulusDims[ FileIO.DIM_INDEX_ROWS ], stimulusDims[ FileIO.DIM_INDEX_COLS ], new File(strOutputPath, "stimRe.csv").getPath());
        saliencyMap.save(strOutputPath);
    }
}
