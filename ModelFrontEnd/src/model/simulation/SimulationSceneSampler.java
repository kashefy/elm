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

import org.shared.array.*;

import java.io.*;
import java.util.*;
import javax.swing.AbstractAction;
import model.attention.AbstractAttention;
import model.attention.AttentionParams;
import model.attention.AttentionSequential;
import model.attention.SaliencyParams;

public class SimulationSceneSampler extends SimulationMNIST{
    
    @Override
    public void learn(){
        
        DataLoaderWeightSet weightLoader = new DataLoaderWeightSet();
        weightLoader.setParams(m_params.getMainOutputDir());
        //weightLoader.setWeightValueFilename("weightsForPreLearn.csv");
        weightLoader.setWeightValueFilename("weights.csv");
        weightLoader.init();
        weightLoader.load();
        DataLoaderWeightSet biasLoader = new DataLoaderWeightSet();
        biasLoader.setParams(m_params.getMainOutputDir());
        //biasLoader.setWeightValueFilename("biasesForPreLearn.csv");
        biasLoader.setWeightValueFilename("biases.csv");
        biasLoader.init();
        biasLoader.load();
        for(int i=0; i<m_nofLearners; i++){
            
            m_arrZNeurons[i].setWeights(weightLoader.getSample(i));
            m_arrZNeurons[i].setBias(biasLoader.getSample(i)[0]);
        }
        
        weightLoader = new DataLoaderWeightSet();
        weightLoader.setParams(m_params.getMainOutputDir());
        //weightLoader.setWeightValueFilename("weightsAuxForPreLearn.csv");
        weightLoader.setWeightValueFilename("weightsAux.csv");
        weightLoader.init();
        weightLoader.load();
        biasLoader = new DataLoaderWeightSet();
        biasLoader.setParams(m_params.getMainOutputDir());
        //biasLoader.setWeightValueFilename("biasesAuxForPreLearn.csv");
        biasLoader.setWeightValueFilename("biasesAux.csv");
        biasLoader.init();
        biasLoader.load();
        m_nofLearnersAux = biasLoader.getNofSamples();
        for(int i=0; i<m_nofLearnersAux; i++){
            
            m_arrZNeuronsAux[i].setWeights(weightLoader.getSample(i));
            m_arrZNeuronsAux[i].setBias(biasLoader.getSample(i)[0]);
        }

    }
    
    @Override
    public void test(){
        
        testSingle();

    }
    
    public void testSingle(){
        
        DataLoaderImageSetCSVTranslation dataLoader = new DataLoaderImageSetCSVTranslation();
        dataLoader.setParams(m_params.getMainInputDir());
        dataLoader.init();
        dataLoader.load();
        
        Random generator = new Random();
        int range = dataLoader.getNofSamples() - m_params.getNofTrainStimuli();
        int starti =  generator.nextInt(range) + m_params.getNofTrainStimuli();
        int endi = starti+1;
        int nofStimuli = endi-starti;
        int gapWidth = m_params.getLearnerParamsRef().getHistoryLength();
        int nofResponsesPerStimulus = m_params.getEncDurationInMilSec()+gapWidth;
         
        int[] arrTestLabelNames = determineLabelSet(0,m_params.getNofTrainStimuli());
        int nofCauses = m_params.getNofCauses();
        
        // membranePotentialZ : nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double [][][] membranePotentialZ = new double [ m_nofLearners ][ nofStimuli ][ nofResponsesPerStimulus ] ;
        int [][] arrResponse = new int [ nofStimuli ][ nofResponsesPerStimulus ];
        int noWinnerCount = 0;
        
        int [] arrLearnerIndicies = new int[ m_nofLearners ];
        for(int i=0; i<m_nofLearners; i++)
            arrLearnerIndicies[i] = i;
        PredictionStats predictionStats = new PredictionStats();
        predictionStats.setParams(1,arrLearnerIndicies);
        predictionStats.init();
        
        int [] arrLearnerIndiciesAll = new int[ m_arrZNeuronsAll.length ];
        for(int i=0; i<m_arrZNeuronsAll.length; i++)
            arrLearnerIndiciesAll[i] = i;
        PredictionStats predictionStatsAll = new PredictionStats();
        predictionStatsAll.setParams(1,arrLearnerIndiciesAll);
        predictionStatsAll.init();

        int [] allZeroSpikeTrain = new int [ m_nofYNeurons ];
        
        SaliencyParams saliencyParams = new SaliencyParams();
        saliencyParams.load(null);
        AttentionParams attentionParams = new AttentionParams();
        AbstractAttention attention = new AttentionSequential();
        attention.setParams(attentionParams);
        attention.init(); 
        
        int [] sceneDims = dataLoader.getDims();
        int nofSceneRows = sceneDims[ FileIO.DIM_INDEX_ROWS ];
        int nofSceneCols = sceneDims[ FileIO.DIM_INDEX_COLS ];
        int [] windowDims = attention.getWindowDims();
        int nofWindowRows = windowDims[ FileIO.DIM_INDEX_ROWS ];
        int nofWindowCols = windowDims[ FileIO.DIM_INDEX_COLS ];
        int nofAttShiftsY = (nofSceneRows - nofWindowRows)/attention.m_shiftY + 1;
        int nofAttShiftsX = (nofSceneCols - nofWindowCols)/attention.m_shiftX + 1;
        int nofAttWindows = nofAttShiftsY * nofAttShiftsX;
        
        double [] arrAvgCondEntropyPerAttWindow = new double [ nofAttWindows ];
        double [] arrAvgFiringSumPerAttWindow = new double [ nofAttWindows ];
        
        double [] arrAvgCondEntropyPerAttWindowAll = new double [ nofAttWindows ];
        double [] arrAvgFiringSumPerAttWindowAll = new double [ nofAttWindows ];
        
        int [][][] arrFiringCountsPerAttWindow = new int [ nofStimuli ][ nofAttWindows ][ m_nofLearners ];
        double [][][] arrFiringProbsPerAttWindow = new double [ nofStimuli ][ nofAttWindows ][ m_nofLearners ];
        
        int [][][] arrFiringCountsPerAttWindowAll = new int [ nofStimuli ][ nofAttWindows ][ m_nofLearnersAux ];
        double [][][] arrFiringProbsPerAttWindowAll = new double [ nofStimuli ][ nofAttWindows ][ m_nofLearnersAux ];
        
        int [] arrWinner = new int [ nofAttWindows ];
        double []arrWinningPercentage = new double [ nofAttWindows ];
        
        int [] arrWinnerAll = new int [ nofAttWindows ];
        double []arrWinningPercentageAll = new double [ nofAttWindows ];
        
        int relSi = 0;
        for(int si=starti; si<endi; si++){                
            int nCurrentLabel = dataLoader.getLabel(si);
            double [] scene = dataLoader.getSample(si);
            FileIO.saveArrayToCSV(scene, nofSceneRows, nofSceneCols, m_params.getMainOutputDir()+"\\att\\scene.csv");
            attention.setScene(scene);
            
            
            for(int ai=0; ai<nofAttWindows; ai++){
                
                double [] stimulus = attention.getWindow();
 
                //FileIO.saveArrayToCSV(stimulus, nofWindowRows, nofWindowCols, m_params.getMainOutputDir()+"\\att\\" +ai+ ".csv");
                
                int rep = 0;
                while(rep++ < 1){
                    
                    int [][] spikesY = m_encoder.encode(stimulus);
                    //int [][] temp = ModelUtils.pad2D(spikesY, 6, ModelUtils.MODE_PAD_COLS);

                    for(int yt=0; yt<nofResponsesPerStimulus; yt++){

                        // create array for spikes of all y neurons at time yt
                        // traverse through columns of spikesY[][]                    
                        int [] spikesYAtT = (yt < m_params.getEncDurationInMilSec())? ModelUtils.extractColumns(spikesY, yt) : allZeroSpikeTrain;

                        for(int zi=0; zi<m_nofLearners; zi++){

                            membranePotentialZ[zi][ relSi ][ yt ] = m_arrZNeurons[zi].predict(spikesYAtT);
                        }
                        for(int zi=0; zi<m_nofLearnersAux; zi++){

                            m_arrZNeuronsAux[zi].predict(spikesYAtT);
                        }

                        int wtaResponse = m_wta.compete();
                        if(wtaResponse != AbstractCompetition.WTA_NONE){

                            if(wtaResponse == AbstractCompetition.WTA_ALL)
                                wtaResponse = PredictionStats.RESPONSE_ALL;
                            predictionStats.addResponse(0, wtaResponse);
                        }
                        else 
                            noWinnerCount++;
                        
                        arrResponse[ relSi ][ yt ] = wtaResponse;
                        
                        int wtaResponseAll = m_wtaAll.compete();
                        if(wtaResponseAll != AbstractCompetition.WTA_NONE){

                            if(wtaResponseAll == AbstractCompetition.WTA_ALL)
                                wtaResponseAll = PredictionStats.RESPONSE_ALL;
                            predictionStatsAll.addResponse(0, wtaResponseAll);
                        }
                    }
                }
                
                int [][] firingCounts = predictionStats.getFiringCounts();
                arrFiringCountsPerAttWindow[relSi][ai] = firingCounts[0];
                
                int [] firingSums = new int [ 1 ];
                double avgFiringSum = predictionStats.calcFiringSums(firingSums);
                arrAvgFiringSumPerAttWindow[ ai ] = avgFiringSum;
                
                double [][] firingProbs = predictionStats.calcFiringProbs();
                arrFiringProbsPerAttWindow[relSi][ai] = firingProbs[0];

                double [] arrCondEntropy = new double [ 1 ];
                double avgCondEntropy = predictionStats.calcConditionalEntropy(arrCondEntropy);
                arrAvgCondEntropyPerAttWindow[ ai ] = avgCondEntropy;
                
                int winnerPerWindow = predictionStats.findMax()[0];
                arrWinner[ ai ] = winnerPerWindow;
                arrWinningPercentage[ ai ] = firingProbs[0][ winnerPerWindow ];
                
                predictionStats.reset();
                
                firingCounts = predictionStatsAll.getFiringCounts();
                arrFiringCountsPerAttWindowAll[relSi][ai] = firingCounts[0];
                
                firingSums = new int [ 1 ];
                avgFiringSum = predictionStatsAll.calcFiringSums(firingSums);
                arrAvgFiringSumPerAttWindowAll[ ai ] = avgFiringSum;
                
                firingProbs = predictionStatsAll.calcFiringProbs();
                arrFiringProbsPerAttWindowAll[relSi][ai] = firingProbs[0];

                arrCondEntropy = new double [ 1 ];
                avgCondEntropy = predictionStatsAll.calcConditionalEntropy(arrCondEntropy);
                arrAvgCondEntropyPerAttWindowAll[ ai ] = avgCondEntropy;
                
                winnerPerWindow = predictionStatsAll.findMax()[0];
                arrWinnerAll[ ai ] = winnerPerWindow;
                arrWinningPercentageAll[ ai ] = firingProbs[0][ winnerPerWindow ];
                
                predictionStatsAll.reset();
                
                attention.attend(null);
            }
            relSi++;
            
            
        }    
        
        // for speicalized learners
        FileIO.saveArrayToCSV(arrAvgCondEntropyPerAttWindow, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"wAvgCondEntropy.csv");
        FileIO.saveArrayToCSV(arrAvgFiringSumPerAttWindow, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"wAvgFiringSum.csv");
        
        FileIO.saveArrayToCSV(arrFiringCountsPerAttWindow[0], m_params.getMainOutputDir()+"wFiringCounts.csv");
        FileIO.saveArrayToCSV(arrFiringProbsPerAttWindow[0], m_params.getMainOutputDir()+"wFiringProbs.csv");

        //ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response.csv", arrResponse);   
        
        FileIO.saveArrayToCSV(arrWinner, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"winnerPerWindow.csv");
        FileIO.saveArrayToCSV(arrWinningPercentage, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"winningPerc.csv");
       
        // for all
        FileIO.saveArrayToCSV(arrAvgCondEntropyPerAttWindowAll, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"wAvgCondEntropyAll.csv");
        FileIO.saveArrayToCSV(arrAvgFiringSumPerAttWindowAll, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"wAvgFiringSumAll.csv");
        
        FileIO.saveArrayToCSV(arrFiringCountsPerAttWindowAll[0], m_params.getMainOutputDir()+"wFiringCountsAll.csv");
        FileIO.saveArrayToCSV(arrFiringProbsPerAttWindowAll[0], m_params.getMainOutputDir()+"wFiringProbsAll.csv");

        //ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response.csv", arrResponse);   
        
        FileIO.saveArrayToCSV(arrWinnerAll, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"winnerPerWindowAll.csv");
        FileIO.saveArrayToCSV(arrWinningPercentageAll, nofAttShiftsY, nofAttShiftsX, m_params.getMainOutputDir()+"winningPercAll.csv");
       
    }
    
    public SimulationSceneSampler(){
        
    }
}
