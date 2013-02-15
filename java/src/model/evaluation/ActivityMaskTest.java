/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.evaluation;

/**
 *
 * @author woodstock
 */
import model.attention.*;
import model.utils.files.*;
import java.io.File;

public class ActivityMaskTest {
    
    public static int test_activity_sub_windows(){
        
        String strMainInputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\input\\MNIST\\";
        File mainInputDir = new File(strMainInputDir);
        String strMainOutputDir = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\";
        File mainOutputDir = new File(strMainOutputDir);
        String strActivity = "activity\\";
        File attOutputDir = new File(mainOutputDir, strActivity);
        
        DataLoaderImageSetCSV dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strMainInputDir);
        dataLoader.init();
        dataLoader.load();
        double[] stimulus = dataLoader.getSample(25);
       
        int [] stimulusDims = dataLoader.getDims();
        int nof_scene_rows = stimulusDims[ FileIO.DIM_INDEX_ROWS ];
        int nof_scene_cols = stimulusDims[ FileIO.DIM_INDEX_COLS ];
        
        AttentionParams attentionParams = new AttentionParams();
        attentionParams.load(new File(mainInputDir, "attentionParamFile.yml").getPath());
        
        AbstractAttention attention = new AttentionSalient();
        attention.setParams(attentionParams);
        attention.init();
        
        ActivityMask activity_mask = new ActivityMask();
        activity_mask.setParams(nof_scene_rows, nof_scene_cols);
        //activity_mask.set_lower_threshold_excl(0.05);
        activity_mask.init();
        
        attention.setScene(stimulus);
        
        int nofAttentions = 500000;
        
        int [] window_dims = attention.getWindowDims();
        
        for(int ai=0; ai<nofAttentions; ai++){
        
            attention.attend(null);
            
            int [] window_loc = attention.getWindowLoc();
            
            double[] window = attention.getWindow();
            activity_mask.addSample(window, window_loc, window_dims);
        }
        double[] activity_intensity = activity_mask.calc_activity_intensity();
        FileIO.saveArrayToCSV(activity_intensity, nof_scene_rows, nof_scene_cols, new File(attOutputDir, "activity_mask_sub_windows.csv").getPath());
        return 0;
    }
}
