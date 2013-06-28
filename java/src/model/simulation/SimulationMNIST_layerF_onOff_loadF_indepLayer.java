/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.simulation;

/**
 *
 * @author woodstock
 */
import model.*;
import model.features.*;
import model.encoding.*;
import model.neuron.*;
import model.competition.*;
import model.evaluation.*;
import model.utils.*;
import model.utils.files.*;

import java.io.*;
import java.lang.reflect.Array;
import java.util.*;
import model.attention.AbstractAttention;
import model.attention.AttentionSalient;
import model.utils.files.FileIO;
import org.shared.array.RealArray;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.highgui.Highgui;
import org.opencv.core.CvType;
import java.nio.ByteBuffer;

public class SimulationMNIST_layerF_onOff_loadF_indepLayer extends SimulationMNIST_layerF_onOff{
    
    protected boolean m_bload_layer_f;
    
    protected double m_delta_t_mil_sec;
    protected int m_nof_responses_per_window_y2f;
    protected int m_nof_responses_per_stimulus_y2f;
    protected int m_nof_responses_per_stimulus_f2Z;
    
    protected int m_logger_cache_size;
    
    public void init(){
        
        super.init();
        m_bload_layer_f = m_params.is_load_layerF();
        if(m_bload_layer_f){
            DataLoaderWeightSet weightLoader = new DataLoaderWeightSet();
            weightLoader.setParams(m_params.getMainOutputDir());
            weightLoader.setWeightValueFilename("weights_layerF.csv");
            weightLoader.init();
            weightLoader.load();
            DataLoaderWeightSet biasLoader = new DataLoaderWeightSet();
            biasLoader.setParams(m_params.getMainOutputDir());
            biasLoader.setWeightValueFilename("biases_layerF.csv");
            biasLoader.init();
            biasLoader.load();

            for(int li=0; li<m_nofLearners_layerF; ++li){

                m_arrZNeurons_layerF[li].setWeights( weightLoader.getSample(li) );
                m_arrZNeurons_layerF[li].setBias( biasLoader.getSample(li)[0] );
            }
        }
        
        m_delta_t_mil_sec = m_params.getDeltaT()*1000;
        m_nof_responses_per_window_y2f = (int)(m_params.getEncDurationInMilSec()/m_delta_t_mil_sec);
        m_nof_responses_per_stimulus_y2f = m_nof_responses_per_window_y2f;
        m_nof_responses_per_stimulus_f2Z = m_nof_responses_per_stimulus_y2f;
        
        m_logger_cache_size = 4000;
    }
    
    public void learn(){
        
        // predict - compete - update
        int nofStimuli = m_params.getNofTrainStimuli();
        
        DataLoggerInt respsonse_1D_logger_layerZ = new DataLoggerInt();
        respsonse_1D_logger_layerZ.set_params(new File(m_params.getMainOutputDir(), "response_1D_layerZ_learn.csv").getPath(), 1, m_logger_cache_size);
        respsonse_1D_logger_layerZ.init();
        
        DataLogger membrane_pot_logger_layerZ = new DataLogger();
        membrane_pot_logger_layerZ.set_params(new File(m_params.getMainOutputDir(), "membrane_pot_layerZ_learn.dat").getPath(), 
                m_nofLearners_layerZ, m_logger_cache_size, false, true);
        membrane_pot_logger_layerZ.init();
        
        DataLoggerInt respsonse_logger_layerF = new DataLoggerInt();
        respsonse_logger_layerF.set_params(new File(m_params.getMainOutputDir(), "response_layerF_learn.csv").getPath(), m_nofLearners_layerF, m_logger_cache_size);
        respsonse_logger_layerF.init();
        
        DataLoggerInt respsonse_1D_logger_layerF_label = new DataLoggerInt();
        respsonse_1D_logger_layerF_label.set_params(new File(m_params.getMainOutputDir(), "response_1D_layerF_label_learn.csv").getPath(), 1, m_logger_cache_size);
        respsonse_1D_logger_layerF_label.init();
        
        // activity mask will be reduced to mask weights of nodes with low activity
        int[] arrTrainLabelNames = m_dataLoader.determineLabelSet(0, nofStimuli-1);
        int nofCauses = m_params.getNofCauses();
        //int nofMasks = nofCauses;
//        ActivityMask [] arrActivityMask_layerZ = new ActivityMask[ nofMasks_layerZ ];
//        for(int mi=0; mi<nofMasks; mi++){
//            
//            arrActivityMask[mi] = new ActivityMask();
//            arrActivityMask[mi].setParams( m_nofRows*m_nofCols );
//            arrActivityMask[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
//            arrActivityMask[mi].init();
//        }
        
        // evaluation
        int predictionStatWindowSize = m_params.get_predictionStatWindowSize();
        
        EvaluationParams evaluation_params = new EvaluationParams();
        evaluation_params.set_nofLearners(m_nofLearners_layerF);
        evaluation_params.set_label_names(arrTrainLabelNames);
        evaluation_params.set_predictionStats_windowSize(predictionStatWindowSize);
                
        PredictionStats predictionStats_perClass_layerF = new PredictionStats();
        predictionStats_perClass_layerF.setParams(evaluation_params);
        predictionStats_perClass_layerF.init();
        
        PredictionStats predictionStats_layerZ = new PredictionStats();
        evaluation_params.set_nofLearners(m_nofLearners_layerZ);
        predictionStats_layerZ.setParams(evaluation_params);
        predictionStats_layerZ.init();
        
        int [] arr_layerF_names = new int [ m_nofLearners_layerF+1 ]; // +1 for no F fires
        for(int fi=0; fi<m_nofLearners_layerF; ++fi){
            arr_layerF_names[fi] = fi;
        }
        arr_layerF_names[ m_nofLearners_layerF ] = AbstractCompetition.WTA_NONE;
        Arrays.sort(arr_layerF_names);
        
        EvaluationParams evaluation_params_perAttWin = new EvaluationParams();
        evaluation_params_perAttWin.set_nofLearners(1);
        evaluation_params_perAttWin.set_label_names(arr_layerF_names);
        evaluation_params_perAttWin.set_predictionStats_windowSize(m_nof_responses_per_window_y2f);
        
        PredictionStats predictionStats_perAttWin_layerF = new PredictionStats();
        predictionStats_perAttWin_layerF.setParams(evaluation_params_perAttWin);
        predictionStats_perAttWin_layerF.init();
        
        File watch_dir = new File(m_params.getMainOutputDir(), FileIO.DIR_NAME_WATCH);
        DataLogger[] weight_watch_logger_layerZ = null;
        if(m_params.is_log_weight_watch_layerZ()){
            weight_watch_logger_layerZ = new DataLogger[m_nofLearners_layerZ];
            for(int li=0; li<m_nofLearners_layerZ; ++li){
                weight_watch_logger_layerZ[li] = new DataLogger();
                weight_watch_logger_layerZ[li].set_params(new File(watch_dir, "weightWatch_layerZ_"+li+".dat").getPath(), 
                        m_arrZNeurons[li].getNofEvidence(), m_logger_cache_size, false, true);
                weight_watch_logger_layerZ[li].init();
            }
        }
        
        DataLogger bias_watch_logger_layerZ = new DataLogger();
        bias_watch_logger_layerZ.set_params(new File(watch_dir, "bias_watch_layerZ.dat").getPath(), m_nofLearners_layerZ);
        bias_watch_logger_layerZ.init();
        
        DataLogger[] weight_watch_logger_layerF = null;
        if(m_params.is_log_weight_watch_layerF()){
            weight_watch_logger_layerF = new DataLogger[m_nofLearners_layerF];
            for(int li=0; li<m_nofLearners_layerF; ++li){
                weight_watch_logger_layerF[li] = new DataLogger();
                weight_watch_logger_layerF[li].set_params(new File(watch_dir, "weightWatch_layerF_"+li+".dat").getPath(), 
                        m_arrZNeurons_layerF[li].getNofEvidence(), m_logger_cache_size, false, true);
                weight_watch_logger_layerF[li].init();
            }
        }
        
        DataLogger bias_watch_logger_layerF = new DataLogger();
        bias_watch_logger_layerF.set_params(new File(watch_dir, "bias_watch_layerF.dat").getPath(),
                m_nofLearners_layerF, m_logger_cache_size, false, true);
        bias_watch_logger_layerF.init();
        
        DataLoggerInt spikes_f2Z_logger = new DataLoggerInt();
        spikes_f2Z_logger.set_params(new File(m_params.getMainOutputDir(), "spikes_f2Z.csv").getPath(), m_arrZNeurons[0].getNofEvidence(), m_logger_cache_size);
        spikes_f2Z_logger.init();
        
        for(int si=0; si<nofStimuli; ++si){
            
            // transform input into set of spike trains
            m_dataLoader.load(); // load here if DataLoaderImageSetCSV_incremental()
            int nCurrent_label = m_dataLoader.getLabel(si);
            if (nCurrent_label == DataLoaderImageSetCSV.INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(nCurrent_label) + "at i=" + Integer.toString(si);
                System.err.println(message);
            }
            double [] stimulus = m_dataLoader.getSample(si);
            
//            String filename = String.format(".\\x\\s-%04d", si)+".png";
//            FileIO.saveArrayToImg(stimulus, m_attention.getSceneDims()[FileIO.DIM_INDEX_COLS], m_attention.getSceneDims()[FileIO.DIM_INDEX_ROWS], filename);
            m_attention.setScene(stimulus);
            //m_attention.save(".\\x\\");
            
            int [][] spikes_f_per_stimulus = new int [ m_nofLearners_layerF ][ m_nof_responses_per_stimulus_y2f ];
            
            for(int ai=0; ai<m_nofAttentions; ++ai){
                
                clear_layer_history(m_arrZNeurons_layerF);
                
                m_attention.attend(null);
                double [] window_of_attention = m_attention.getWindow();
                
//                filename = String.format(".\\x\\s-%04d", si)+"_"+ai+".png";
//                FileIO.saveArrayToImg(window_of_attention,
//                        m_attention.getWindowDims()[FileIO.DIM_INDEX_ROWS], 
//                        m_attention.getWindowDims()[FileIO.DIM_INDEX_COLS], filename);
                
                int [][] spikes_y_p1 = m_params.is_do_orient()? m_encoder.encode(window_of_attention) : null;

                int [][] spikes_y_p2 = m_params.is_do_intensity()? m_encoder_onOff.encode(window_of_attention) : null;

                // for each column in all spike trains of layer F
                // predict - compete - update
                double yt=0;
                for(int yti=0; yti<m_nof_responses_per_window_y2f; ++yti, yt+=m_delta_t_mil_sec){

                    // create array for spikes of all y neurons at time yi
                    // traverse through columns of spikes_y[][]
                    int [] spikes_atT_in;
                    int [] spikes_atT_out;
                    
                    spikes_atT_in = new int [m_nofYNeurons];
                    int [] spikes_atT_in_p1;
                    if(m_params.is_do_orient()){
                        spikes_atT_in_p1 = ModelUtils.extract_columns(spikes_y_p1, yti);
                        System.arraycopy(spikes_atT_in_p1, 0, spikes_atT_in, 0, m_encoder.getNofEncoderNodes());
                    }

                    int [] spikes_atT_in_p2;
                    if(m_params.is_do_intensity()){
                       spikes_atT_in_p2 = ModelUtils.extract_columns(spikes_y_p2, yti);
                       System.arraycopy(spikes_atT_in_p2, 0, spikes_atT_in, m_nofYNeurons-m_encoder_onOff.getNofEncoderNodes(), m_encoder_onOff.getNofEncoderNodes());
                    }

                    // predict F
                    
                    //System.out.println(Arrays.toString(spikes_atT_in));
                    for(int li=0; li<m_nofLearners_layerF; ++li){

                        double membranePotential_layerF = m_arrZNeurons_layerF[li].predict(spikes_atT_in);
                        //System.out.print(membranePotential_layerF+" ");
                    }//System.out.println();

                    // let F's compete before updating
                    int wta_response = m_wta_layerF.compete();
                    
                    if(wta_response != AbstractCompetition.WTA_NONE){

                        int [] arr_wta_response;
                        arr_wta_response = m_wta_layerF.getOutcome();
                        for(int li=0; li<m_nofLearners_layerF; ++li){

                            m_arrZNeurons_layerF[ li ].letFire(arr_wta_response[ li ]==1);
                        }
                        spikes_atT_out = arr_wta_response;
                        
                        predictionStats_perClass_layerF.addResponse( wta_response, nCurrent_label );
                        predictionStats_perAttWin_layerF.addResponse( 0, wta_response );
                    }
                    else{
                        spikes_atT_out = new int [ m_nofLearners_layerF ];
                    }
                    
                    // update F // unless pre-learned and loaded from file
                    double [] arr_bias_layer_F = new double[m_nofLearners_layerF];
                    for(int li=0; li<m_nofLearners_layerF; ++li){

                        if(m_params.is_log_weight_watch_layerF())
                            weight_watch_logger_layerF[li].add_sample(m_arrZNeurons_layerF[li].getWeights());
                        arr_bias_layer_F[li] = m_arrZNeurons_layerF[li].getBias();
                        if(!m_bload_layer_f){
                            m_arrZNeurons_layerF[li].update();
                        }
                    }
                    bias_watch_logger_layerF.add_sample(arr_bias_layer_F);
                    
                    // merge spikes from different windows of same stimulus
                    if(ai > 0){
                        int [] spikes_atT_pre = ModelUtils.extract_columns(spikes_f_per_stimulus, yti);
                        for(int li=0; li<spikes_atT_pre.length; ++li){
                            int val_li = spikes_atT_pre[li];
                            spikes_atT_out[li] = (val_li == 0)? spikes_atT_out[li] : val_li;
                        }
                    }
                    
                    ModelUtils.insert_column(spikes_atT_out, spikes_f_per_stimulus, yti);
                }
            }
            
            for(int i=0; i<m_nof_responses_per_stimulus_y2f; ++i){
                int [] tmp = ModelUtils.extract_columns(spikes_f_per_stimulus, i);
                respsonse_logger_layerF.add_sample(tmp);
            }
            int [] tmp = {nCurrent_label};
            respsonse_1D_logger_layerF_label.add_sample(tmp);
            
            int [][] spikes_f2z;
            spikes_f2z = refactor_spike_train_layerF(spikes_f_per_stimulus);
            
            clear_layer_history(m_arrZNeurons);
            
            for(int ft=0; ft<m_nof_responses_per_stimulus_f2Z; ++ft){

                // create array for f spikes at time t
                // traverse through columns of spikes_f[][]
                int [] spikes_atT;
                spikes_atT = ModelUtils.extract_columns(spikes_f2z, ft);
                
                spikes_f2Z_logger.add_sample(spikes_atT);

                // predict Z
                double [] arr_membrane_pot_layerZ = new double[m_nofLearners_layerZ];
                for(int li=0; li<m_nofLearners_layerZ; ++li){

                    double membrane_potential = m_arrZNeurons[li].predict(spikes_atT);
                    arr_membrane_pot_layerZ[li] = membrane_potential;
                }
                membrane_pot_logger_layerZ.add_sample(arr_membrane_pot_layerZ);
                
                // let Z's compete before updating
                int wta_response = m_wta.compete();
                if(wta_response != AbstractCompetition.WTA_NONE){

                    int [] arr_wta_response;
                    arr_wta_response = m_wta.getOutcome();
                    for(int li=0; li<m_nofLearners_layerZ; ++li){

                        m_arrZNeurons[ li ].letFire(arr_wta_response[ li ]==1);
                    }
                    predictionStats_layerZ.addResponse( wta_response, nCurrent_label );
                }

                // update Z
                double [] arr_bias_layerZ = new double[m_nofLearners_layerZ];
                for(int li=0; li<m_nofLearners_layerZ; ++li){
                    
                    if(m_params.is_log_weight_watch_layerZ())
                        weight_watch_logger_layerZ[li].add_sample(m_arrZNeurons[li].getWeights());
                    arr_bias_layerZ[li] = m_arrZNeurons[li].getBias();
                    m_arrZNeurons[li].update();               
                }
                bias_watch_logger_layerZ.add_sample(arr_bias_layerZ);
                int [] tmp2 = {wta_response};
                respsonse_1D_logger_layerZ.add_sample(tmp2);
            }
        }
        ModelPredictionTest.saveWeights(new File( m_params.getMainOutputDir(), "weights_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        ModelPredictionTest.saveBiases(new File( m_params.getMainOutputDir(), "biases_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        
        ModelPredictionTest.saveWeights(new File( m_params.getMainOutputDir(), "weights_layerZ.csv" ).getPath(), m_arrZNeurons);
        ModelPredictionTest.saveBiases(new File( m_params.getMainOutputDir(), "biases_layerZ.csv" ).getPath(), m_arrZNeurons);
        
        respsonse_1D_logger_layerZ.flush();
        respsonse_logger_layerF.flush();
        respsonse_1D_logger_layerF_label.flush();
        spikes_f2Z_logger.flush();
        membrane_pot_logger_layerZ.flush();
//        for(int mi=0; mi<nofMasks; mi++){
//            arrActivityMask[mi].calc_activity_intensity();
//        }
//        String strMaskFile = m_params.getMainOutputDir()+"masksClasses.csv";
//        ModelPredictionTest.saveMasks(strMaskFile, arrActivityMask);
        
        double [] arrAvgCondEntropy;
        arrAvgCondEntropy = predictionStats_perClass_layerF.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy_perClass_layerF.csv").getPath() );
        
        arrAvgCondEntropy = predictionStats_perAttWin_layerF.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy_perAttWin_layerF.csv").getPath() );
            
        arrAvgCondEntropy = predictionStats_layerZ.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy_layerZ.csv").getPath() );
        
        for(int li=0; li<m_nofLearners_layerZ && m_params.is_log_weight_watch_layerZ(); ++li)
            weight_watch_logger_layerZ[li].flush();
        bias_watch_logger_layerZ.flush();
        
        for(int li=0; li<m_nofLearners_layerF && m_params.is_log_weight_watch_layerF(); ++li)
            weight_watch_logger_layerF[li].flush();
        bias_watch_logger_layerF.flush();
    }
    
    public void test(){
        
        m_wta.refToLearners(m_arrZNeurons);
        
        int starti =  m_params.getNofTrainStimuli();
        int endi = m_totalNofStimuli;
        int nofStimuli = endi-starti+1;
        
        int[] arrTestLabelNames = m_dataLoader.determineLabelSet(starti, endi);
        int nofCauses = m_params.getNofCauses();
        
        File dir_activity_layerF = new File(m_params.getMainOutputDir(), FileIO.DIR_NAME_ACTIVITY_LAYER_F);
        int nofMasks_layerF = m_nofLearners_layerF;
        ActivityMask [] arr_activity_mask_layerF = new ActivityMask[ nofMasks_layerF ];
        for(int mi=0; mi<nofMasks_layerF; ++mi){
            
            arr_activity_mask_layerF[mi] = new ActivityMask();
            arr_activity_mask_layerF[mi].setParams(m_nofRows*m_nofCols);
            arr_activity_mask_layerF[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arr_activity_mask_layerF[mi].enable_logging( dir_activity_layerF.getPath(), Integer.toString(mi) );
            arr_activity_mask_layerF[mi].init();
        }
        
        int [] scene_dims = m_attention.getSceneDims();
        int nof_scene_rows = scene_dims[ FileIO.DIM_INDEX_ROWS ];
        int nof_scene_cols = scene_dims[ FileIO.DIM_INDEX_COLS ];
        
        File dir_activity_layerZ = new File(m_params.getMainOutputDir(), FileIO.DIR_NAME_ACTIVITY_LAYER_Z);
        int nofMasks_layerZ = m_nofLearners_layerZ;
        ActivityMask [] arr_activity_mask_layerZ = new ActivityMask[ nofMasks_layerZ ];
        for(int mi=0; mi<nofMasks_layerZ; ++mi){
            
            arr_activity_mask_layerZ[mi] = new ActivityMask();
            arr_activity_mask_layerZ[mi].setParams(nof_scene_rows, nof_scene_cols );
            arr_activity_mask_layerZ[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arr_activity_mask_layerZ[mi].enable_logging( dir_activity_layerZ.getPath(), Integer.toString(mi) );
            arr_activity_mask_layerZ[mi].init();
        }
        
        // membranePotential_layerF : nofZ,nofstimuli,nofAttentios,durationOfSpikeTrain Y for each stimulus per F
        //double [][][][] arr_membranePotential_layerF = new double [ m_nofLearners_layerF ][ nofStimuli ][ m_nofAttentions ][ m_nof_responses_per_window_y2f ] ;
        //int [][][] arrResponse_layerF = new int [ nofStimuli ][ m_nofAttentions ][ m_nof_responses_per_window_y2f ];
        
        // membrane_pot_layerZ : nofZ,nofstimuli,durationOfSpikeTrain F for each stimulus per Z
        //double [][] membrane_pot_layerZ = new double [ m_nofLearners_layerZ ][ nofStimuli*m_nof_responses_per_stimulus_f2Z ] ;
        //int [][] arrResponse_layerZ = new int [ nofStimuli ][ m_nof_responses_per_stimulus_f2Z ];
        
        int noWinnerCount_layerF = 0;
        int noWinnerCount_layerZ = 0;
        
        PredictionStats predictionStats_layerF = new PredictionStats();
        predictionStats_layerF.setParams(m_nofLearners_layerF, arrTestLabelNames);
        predictionStats_layerF.init();
        
        PredictionStats predictionStats = new PredictionStats();
        predictionStats.setParams(m_nofLearners_layerZ, arrTestLabelNames);
        predictionStats.init();
        
        int [] arr_layerF_names = new int [ m_nofLearners_layerF+1 ]; // +1 for no F fires
        for(int fi=0; fi<m_nofLearners_layerF; ++fi){
            arr_layerF_names[fi] = fi;
        }
        arr_layerF_names[ m_nofLearners_layerF ] = AbstractCompetition.WTA_NONE;
        Arrays.sort(arr_layerF_names);
        
        PredictionStats predictionStats_F2Z = new PredictionStats();
        predictionStats_F2Z.setParams(m_nofLearners_layerZ, arr_layerF_names);
        predictionStats_F2Z.init();
        
        for(int si=starti, relSi=0; si<endi; si++, ++relSi){
            
            m_dataLoader.load(); // load here if DataLoaderImageSetCSV_incremental()
            int nCurrentLabel = m_dataLoader.getLabel(si);
            if (nCurrentLabel == DataLoaderImageSetCSV.INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(nCurrentLabel) + "at i=" + Integer.toString(si);
                System.err.println(message);
            }
            double [] stimulus = m_dataLoader.getSample(si);
            m_attention.setScene(stimulus);
            
            int [][] spikes_f_per_stimulus = new int [ m_nofLearners_layerF ][ m_nof_responses_per_stimulus_y2f ];
            
            double [][] arr_window_of_attention = new double[ m_nofAttentions ][ m_nofRows*m_nofCols ];
            
            for(int ai=0; ai<m_nofAttentions; ++ai){
                
                clear_layer_history(m_arrZNeurons_layerF);
                
                m_attention.attend(null);
                double [] window_of_attention = m_attention.getWindow();
                System.arraycopy(window_of_attention, 0, arr_window_of_attention[ai], 0, m_nofRows*m_nofCols);
                
                int [][] spikes_y_p1 = m_params.is_do_orient()? m_encoder.encode(window_of_attention) : null;

                int [][] spikes_y_p2 = m_params.is_do_intensity()? m_encoder_onOff.encode(window_of_attention) : null;
                
                int yt = 0;
                for(int yti=0; yti<m_nof_responses_per_window_y2f; ++yti, yt+=m_delta_t_mil_sec){

                    // create array for spikes of all y neurons at time yi
                    // traverse through columns of spikes_y[][]                    
                    int [] spikes_atT_in;
                    int [] spikes_atT_out;

                    spikes_atT_in = new int [m_nofYNeurons];
                    int [] spikes_atT_in_p1;
                    if(m_params.is_do_orient()){
                        spikes_atT_in_p1 = ModelUtils.extract_columns(spikes_y_p1, yti);
                        System.arraycopy(spikes_atT_in_p1, 0, spikes_atT_in, 0, m_encoder.getNofEncoderNodes());
                    }

                    int [] spikes_atT_in_p2;
                    if(m_params.is_do_intensity()){
                       spikes_atT_in_p2 = ModelUtils.extract_columns(spikes_y_p2, yti);
                       System.arraycopy(spikes_atT_in_p2, 0, spikes_atT_in, m_nofYNeurons-m_encoder_onOff.getNofEncoderNodes(), m_encoder_onOff.getNofEncoderNodes());
                    }

                    for(int li=0; li<m_nofLearners_layerF; ++li){

                        double membranePotential_layerF = m_arrZNeurons_layerF[li].predict(spikes_atT_in);
                        //arr_membranePotential_layerF[fi][ relSi ][ai][ yi ] = membranePotential_layerF;
                    }

                    // let F neurons compete
                    int wta_response_layerF = m_wta_layerF.compete();
                    //System.out.print(wta_response_layerF + " ");
                    if( wta_response_layerF != AbstractCompetition.WTA_NONE ){

                        int [] arr_wta_response_layerF = m_wta_layerF.getOutcome();
                        
                        if( wta_response_layerF == AbstractCompetition.WTA_ALL )
                            wta_response_layerF = PredictionStats.RESPONSE_ALL;
                        predictionStats_layerF.addResponse( wta_response_layerF, nCurrentLabel );
                        arr_activity_mask_layerF[ wta_response_layerF ].add_sample( window_of_attention );
                        
                        spikes_atT_out = arr_wta_response_layerF;
                    }
                    else{
                        
                        spikes_atT_out = new int[m_nofLearners_layerF];
                        noWinnerCount_layerF++;
                    }
                    
                    // merge spikes from different windows of same stimulus
                    if(ai > 0){
                        int [] spikes_atT_pre = ModelUtils.extract_columns(spikes_f_per_stimulus, yti);
                        for(int li=0; li<spikes_atT_pre.length; ++li){
                            int val_li = spikes_atT_pre[li];
                            spikes_atT_out[li] = (val_li == 0)? spikes_atT_out[li] : val_li;
                        }
                    }
                    ModelUtils.insert_column(spikes_atT_out, spikes_f_per_stimulus, yti);
                    //arrResponse_layerF[ relSi ][ai][ yti ] = wta_response_layerF;
                }
            }

            int [][] spikes_f2z;
            spikes_f2z = refactor_spike_train_layerF(spikes_f_per_stimulus);
            
            clear_layer_history(m_arrZNeurons);
            
            for(int ft=0; ft<m_nof_responses_per_stimulus_f2Z; ++ft){

                // create array for f spikes at time t
                // traverse through columns of spikes_f[][]
                int [] spikes_atT_in;
                spikes_atT_in = ModelUtils.extract_columns(spikes_f2z, ft);

                for(int li=0; li<m_nofLearners_layerZ; ++li){

                    double membranePotential_layerZ = m_arrZNeurons[li].predict( spikes_atT_in );
                    //membrane_pot_layerZ[li][ relSi*m_nof_responses_per_stimulus_f2Z+ft ] = membranePotential_layerZ;
                }

                // let Z neurons compete
                int wta_response = m_wta.compete();
                if(wta_response != AbstractCompetition.WTA_NONE){

                    if(wta_response == AbstractCompetition.WTA_ALL)
                        wta_response = PredictionStats.RESPONSE_ALL;

                    predictionStats.addResponse( wta_response, nCurrentLabel );
                    boolean layerF_fired = false;
                    for(int fi=0; fi<m_nofLearners_layerF; ++fi){

                        if(spikes_atT_in[fi] == 1){
                            predictionStats_F2Z.addResponse(wta_response, fi);
                            layerF_fired = true;
                        }
                    }
                    if( !layerF_fired ){
                        predictionStats_F2Z.addResponse(wta_response, AbstractCompetition.WTA_NONE);
                    }
                    arr_activity_mask_layerZ[ wta_response ].add_sample( stimulus );
                    //arr_ActivityMask_layerZ[ wta_response ].add_sample( window_of_attention, m_attention.getWindowLoc(), m_attention.getWindowDims() );
                }
                else{
                    noWinnerCount_layerZ++;
                }
                //arrResponse_layerZ[ relSi ][ ft ] = wta_response;
            }
        }    

        for(int mi=0; mi<nofMasks_layerF; ++mi){
            arr_activity_mask_layerF[mi].calc_activity_intensity();
            arr_activity_mask_layerF[mi].flush();
        }
        ModelPredictionTest.saveMasks( new File(m_params.getMainOutputDir(), "masksLearners_layerF.csv").getPath(), arr_activity_mask_layerF );
        
        for(int mi=0; mi<nofMasks_layerZ; ++mi){
            arr_activity_mask_layerZ[mi].calc_activity_intensity();
            arr_activity_mask_layerZ[mi].flush();
        }
        ModelPredictionTest.saveMasks( new File(m_params.getMainOutputDir(), "masksLearners_layerZ.csv").getPath(), arr_activity_mask_layerZ );

        //ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response_layerF.csv", arrResponse_layerF); 
        //ModelPredictionTest.saveResponses(new File(m_params.getMainOutputDir(),"response_layerZ_test.csv").getPath(), arrResponse_layerZ);    
        //FileIO.saveArrayToCSV(membrane_pot_layerZ, new File(m_params.getMainOutputDir(),"membrane_pot_layerZ.csv").getPath());
        
        System.out.println("test set size: "+m_params.getNofTestStimuli());
        
        // print prediction stats of layer F
        int [][] firingCounts = predictionStats_layerF.getFiringCounts();
        int [] arr_firingSums = predictionStats_layerF.calcFiringSums();
        double [][] arr_firing_probs = predictionStats_layerF.calcFiringProbs();
        double [] arr_firingSums_temp = new double[m_nofLearners_layerF];
        double [] arrCondEntropy = new double [ m_nofLearners_layerF ];
        predictionStats_layerF.calcConditionalEntropy(arrCondEntropy);
        
        System.out.println("layer F:");
        for(int li=0; li<m_nofLearners_layerF; ++li){

            for(int ci=0; ci<nofCauses; ++ci){

                System.out.print(firingCounts[li][ci] + "  ");
            }            

            System.out.print("  \u03A3  " + arr_firingSums[li]);
            System.out.print("  condEntr  " + arrCondEntropy[ li ]);
            System.out.println();
            arr_firingSums_temp[li] = (double)arr_firingSums[li];
        }
        System.out.println("No winner count: "+noWinnerCount_layerF);
        // average firing sum during testing:
        RealArray firing_sums = new RealArray(arr_firingSums_temp);
        System.out.format("Mean firing sum= %.0f%n", firing_sums.aMean());
        m_log_results.format("%.0f,", firing_sums.aMean());
        
        // uniformity of firing
        double firing_uniformity = firing_sums.uMul(1.0/firing_sums.aSum()).aEnt();
        System.out.format("Uniformity of firing probs= %.3f%n", firing_uniformity);
        m_log_results.format("%.3f,", firing_uniformity);

        ModelPredictionTest.savePredictionStats(new File(m_params.getMainOutputDir(), "predictionStats_layerF.csv").getPath(), arr_firing_probs, arrCondEntropy);
        
        int[] arr_label_id = predictionStats_layerF.get_label_names();
        FileIO.saveArrayToCSV(arr_label_id, 1, arr_label_id.length, new File(m_params.getMainOutputDir(), "predictionStats_layerF_label_ids.csv").getPath());
        
        // print prediction stats
        firingCounts = predictionStats.getFiringCounts();
        arr_firingSums = predictionStats.calcFiringSums();
        arr_firing_probs = predictionStats.calcFiringProbs();
        arr_firingSums_temp = new double[m_nofLearners_layerZ];
        arrCondEntropy = new double [ m_nofLearners_layerZ ];
        predictionStats.calcConditionalEntropy(arrCondEntropy);
        
        System.out.println("layer Z:");
        for(int li=0; li<m_nofLearners_layerZ; ++li){

            for(int ci=0; ci<nofCauses; ci++){

                System.out.print(firingCounts[li][ci] + "  ");
            }            

            System.out.print("  \u03A3  " + arr_firingSums[li]);
            System.out.print("  condEntr  " + arrCondEntropy[ li ]);
            System.out.println();
            arr_firingSums_temp[li] = (double)arr_firingSums[li];
        }
        System.out.println("No winner: "+noWinnerCount_layerZ);
        // average firing sum during testing:
        firing_sums = new RealArray(arr_firingSums_temp);
        System.out.format("Mean firing sum= %.0f%n", firing_sums.aMean());
        m_log_results.format("%.0f,", firing_sums.aMean());
        
        // uniformity of firing
        firing_uniformity = firing_sums.uMul(1.0/firing_sums.aSum()).aEnt();
        System.out.format("Uniformity of firing probs= %.3f%n", firing_uniformity);
        m_log_results.format("%.3f,", firing_uniformity);
        
        ModelPredictionTest.savePredictionStats(new File(m_params.getMainOutputDir(), "predictionStats.csv").getPath(), arr_firing_probs, arrCondEntropy);
        arr_label_id = predictionStats.get_label_names();
        FileIO.saveArrayToCSV(arr_label_id, 1, arr_label_id.length, new File(m_params.getMainOutputDir(), "predictionStats_label_ids.csv").getPath());
        
        // print prediction stats F2Z
        firingCounts = predictionStats_F2Z.getFiringCounts();
        arr_firingSums = predictionStats_F2Z.calcFiringSums();
        arr_firing_probs = predictionStats_F2Z.calcFiringProbs();
        arr_firingSums_temp = new double[ m_nofLearners_layerZ ];
        arrCondEntropy = new double [ m_nofLearners_layerZ ];
        predictionStats_F2Z.calcConditionalEntropy(arrCondEntropy);
        System.out.println("layer Z:F2Z");
        for(int li=0; li<m_nofLearners_layerZ; ++li){

            for(int ci=0; ci<m_nofLearners_layerF+1; ++ci){// +1 for no F fires

                System.out.print(firingCounts[li][ci] + "  ");
            }            
            System.out.print("  \u03A3  " + arr_firingSums[li]);
            System.out.print("  condEntr  " + arrCondEntropy[ li ]);
            System.out.println();
            arr_firingSums_temp[li] = (double)arr_firingSums[li];
        }
        // average firing sum during testing:
        firing_sums = new RealArray(arr_firingSums_temp);
        System.out.format("Mean firing sum= %.0f.%n", firing_sums.aMean());
        m_log_results.format("%.0f,", firing_sums.aMean());
        
        // uniformity of firing
        firing_uniformity = firing_sums.uMul(1.0/firing_sums.aSum()).aEnt();
        System.out.format("Uniformity of firing probs= %.3f%n", firing_uniformity);
        m_log_results.format("%.3f,", firing_uniformity);
        
        ModelPredictionTest.savePredictionStats(new File(m_params.getMainOutputDir(), "predictionStats_F2Z.csv").getPath(), arr_firing_probs, arrCondEntropy);
        //m_nofLearners -= m_nofLearnersAux;
        arr_label_id = predictionStats_F2Z.get_label_names();
        FileIO.saveArrayToCSV(arr_label_id, 1, arr_label_id.length, new File(m_params.getMainOutputDir(), "predictionStats_F2Z_label_ids.csv").getPath());
        
        m_log_results.println();
        m_log_results.close();
    }
    
    // resample spikes
    private int [][] resample_spike_train(int[][] par_spike_train){
        
        int _T = par_spike_train[0].length;
        double [] spikes_distr = AbstractEncoder.spikes2distr(par_spike_train);
        int [][] resampled = AbstractEncoder.distr2spikes(spikes_distr, _T);
        return resampled;
    }
    
    // re-invent layer F spike train
    // -bi state (m_nofLearners_layerF*2)
    // -ranking per f (m_nofLearners_layerF*m_nofLearners_layerF)
    private int [][] refactor_spike_train_layerF(int[][] par_spike_train){
        
        int _T = par_spike_train[0].length;
        //int n = m_nofLearners_layerF*2 + m_nofLearners_layerF*m_nofLearners_layerF;
        int n = m_nofLearners_layerF*2;
        int [][] result = new int[n][_T];
        
        SimplePopulationCode bi_state_pop_code = new SimplePopulationCode();
        bi_state_pop_code.init(m_nofLearners_layerF, 2);
        
        RankPopulationCode rank_pop_code = new RankPopulationCode();
        rank_pop_code.init(m_nofLearners_layerF, m_nofLearners_layerF);

        for(int ft=0; ft<_T; ++ft){

            int [] spikes_at_T_new = new int [n];
            
            int[] spikes_at_T = ModelUtils.extract_columns(par_spike_train, ft);
            int [] bi_state = bi_state_pop_code.calcStateOfNeurons(spikes_at_T);
            System.arraycopy(bi_state, 0, spikes_at_T_new, 0, bi_state.length);
            
//            double [] membrane_pot = new double[m_nofLearners_layerF];
//            for(int fi=0; fi<m_nofLearners_layerF; ++fi){
//                membrane_pot[fi] = m_arrZNeurons_layerF[fi].getPredictionValue();
//            }
//            double [] rank_double = rank_pop_code.calcStateOfNeurons(membrane_pot);
//            int [] rank = new int[rank_double.length];
//            for(int i=0; i<rank_double.length; ++i){
//                rank[i] = (int)rank_double[i];
//            }
//            System.arraycopy(rank, 0, spikes_at_T_new, bi_state.length, rank.length);
            
            ModelUtils.insert_column(spikes_at_T_new, result, ft);
            
//            System.out.println(Arrays.toString(spikes_at_T));
//            System.out.println(Arrays.toString(bi_state));
//            System.out.println(Arrays.toString(membrane_pot));
//            System.out.println(Arrays.toString(rank));
//            System.out.println(Arrays.toString(spikes_at_T_new));
        }
        
        return result;
    }
    
    static public void clear_layer_history(ZNeuron[] p_layer){
        
        for(int i=0; i<p_layer.length; ++i){
            p_layer[i].resetHistory();
        }
    }
    
}

