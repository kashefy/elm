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

import java.io.*;
import model.utils.files.FileIO;

public class SimulationSimplePatterns extends AbstractSimulation{
              
    // stimuli
    int m_nofRows;
    int m_nofCols;
    int [] m_stimuliDims;
    int m_nofStimElements;
    double [][] m_stimuli;
    
    // feature maps
    OrientationMap m_mapOrient;
    
    // encoding
    Encoder m_encoder;
    
    // prediction, learning, classification
    int m_nofYNeurons; // get value from enocder
    int m_nofLearners;
    ZNeuron [] m_arrZNeurons;
    
    // competition
    AbstractCompetition m_wta;
    
    public void setParams(SimulationParams params){
        
        m_params = params;
 
        /*
        m_strMainOutputDir = params.getMainOutputDir();
        m_strMainInputDir = params.getMainInputDir();

        // data
        m_nofTrainStimuli = params.getNofTrainStimuli();
        m_nofTestStimuli = params.getNofTestStimuli();
        m_nofRows = params.getNofRows();
        m_nofCols = params.getNofCols();

        // encoding parameters
        m_encFrequency = params.getEncFrequency();
        m_deltaT = params.get;
        m_encDurationInMilSec = params.get;
        m_popCodeFanOut = params.get;

        // prediction parameters
        m_nofCauses = params.get;
        m_nofLearners = params.get;

        // parameter objects

        m_learnerParams = params.get;

        m_competitionParams = params.get;

        m_featParams = params.get;
         */
    }
    
    public void init(){
        
        // generate stimuli of simple patterns
        m_nofRows = m_params.getNofRows();
        m_nofCols = m_params.getNofCols();
        m_stimuliDims = new int[]{m_nofRows,m_nofCols};
        m_nofStimElements = m_nofRows * m_nofCols;
        m_stimuli = new double [ m_params.getNofTrainStimuli() ][ m_nofStimElements ];
        
        m_stimuli[0] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,90);
        m_stimuli[1] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,0);
        m_stimuli[2] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,30);
        m_stimuli[3] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,60);
        m_stimuli[4] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,120);
        m_stimuli[5] = OrientationMapTest.genBarStimulusVals(m_nofRows,m_nofCols,m_nofCols/2,m_nofRows/2,2,150);
        
        // feature maps inits       
        m_mapOrient = new OrientationMap(); 
        m_mapOrient.setParams(m_params.getFeatureMapParamsRef());
        m_mapOrient.init();
        
        // encoding inits
        // need to improve init for m_encoder to conform with other inits
        m_encoder = new Encoder();
        m_encoder.setFeatureMap(m_mapOrient);
        m_encoder.init(m_params.getEncFrequency(), m_params.getDeltaT(), m_params.getEncDurationInMilSec(), 
                m_nofRows*m_nofCols*m_mapOrient.getNofFeatureSets(), m_params.getPopCodeFanOut());
        m_encoder.setInputDimensions(m_stimuliDims);
        
        // predictors/classifiers
        m_nofYNeurons = m_encoder.getNofEncoderNodes(); // get value from enocder
        m_params.getLearnerParamsRef().setNofInputNodes(m_nofYNeurons);
        m_nofLearners = m_params.getNofLeaners();
        m_arrZNeurons = new ZNeuron[ m_nofLearners ];       
        
        for(int zi=0; zi<m_nofLearners; zi++){
            
            //arrZNeurons[zi] = new ZNeuron();
            m_arrZNeurons[zi] = new ZNeuronCompact();
            m_arrZNeurons[zi].setParams(m_params.getLearnerParamsRef());
            m_arrZNeurons[zi].init();
        } 
        
        // competition inits
        //wta = new CompetitionWTAOU();
        m_wta = new WTAPoissonRate();
        m_wta.setParams(m_params.getCompetitionParamsRef());
        m_wta.init();
        m_wta.refToLearners(m_arrZNeurons);
        
    }
    
    public void learn(){
        
        // predict - compete - update
        int nofTrainRepititions = 100;
        int nofStimuli = m_params.getNofTrainStimuli();
        int gapWidth = m_params.getLearnerParamsRef().getHistoryLength();
        int nofResponsesPerStimulus = m_params.getEncDurationInMilSec()+gapWidth;
        int [][] arrResponse = new int [ nofStimuli ][ nofTrainRepititions*nofResponsesPerStimulus ];
        
        // activity mask will be reduced to mask weights of nodes with low activity
        int nofMasks = m_params.getNofCauses();
        double activityMaskLowerThresholdExcl = 0.05;
        ActivityMask [] arrActivityMask = new ActivityMask[ nofMasks ];
        for(int mi=0; mi<nofMasks; mi++){
            
            arrActivityMask[mi] = new ActivityMask();
            arrActivityMask[mi].setParams(m_nofRows*m_nofCols);
            arrActivityMask[mi].set_lower_threshold_excl(activityMaskLowerThresholdExcl);
            arrActivityMask[mi].init();
        }
        
        for(int si=0; si<nofStimuli; si++){
            
            int responseIndex = 0;
            for(int rep=0; rep<nofTrainRepititions; rep++){
                
                // transform input into set of spike trains
                int [][] spikesY = m_encoder.encode(m_stimuli[ si ]);
                arrActivityMask[si].add_sample(m_stimuli[ si ]);
                
                // for each column in all spike trains
                // predict - compete - update
                for(int yt=0; yt<nofResponsesPerStimulus; yt++){
                    
                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikesY[][]
                    int [] spikesYAtT;
                    if(yt < m_params.getEncDurationInMilSec()){
                    
                        spikesYAtT = ModelUtils.extract_columns(spikesY, yt);
                    }
                    else
                        spikesYAtT = new int [ m_nofYNeurons ];
                    
                    // predict
                    for(int zi=0; zi<m_nofLearners; zi++){
                        
                        double membranePotential;
                        membranePotential = m_arrZNeurons[zi].predict(spikesYAtT);
                        //System.out.print(membranePotential+ " ");
                    }
                    
                    // let Z's compete before updating
                    int wtaResponse = m_wta.compete();
                    //System.out.println(wtaResponse);
                    
                    arrResponse[ si ][ responseIndex++ ] = wtaResponse;
                    if(wtaResponse != AbstractCompetition.WTA_NONE){

                        int [] arrWTAResponse;
                        arrWTAResponse = m_wta.getOutcome();
                        for(int zi=0; zi<m_nofLearners; zi++){
                        
                            m_arrZNeurons[ zi ].letFire(arrWTAResponse[ zi ]==1);
                            //    m_arrZNeurons[ zi ].letFire(false);
                        }
                    }

                    // update
                    for(int zi=0; zi<m_nofLearners; zi++){
                        
                        m_arrZNeurons[zi].update();               
                    }
                }  
            }    
        }        
        
        
        for(int mi=0; mi<nofMasks; mi++){
            arrActivityMask[mi].calc_activity_intensity();
        }
        String strMaskFile = m_params.getMainOutputDir()+"masks.csv";
        ModelPredictionTest.saveMasks(strMaskFile, arrActivityMask);
    }
    
    public void test(){
        
        int nofTestRepititions = 100;
        int nofStimuli = m_params.getNofTestStimuli();
        int gapWidth = m_params.getLearnerParamsRef().getHistoryLength();
        int nofResponsesPerStimulus = m_params.getEncDurationInMilSec()+gapWidth;
        
        // membranePotentialZ : nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double [][][] membranePotentialZ = new double [ m_nofLearners ][ nofStimuli ][ nofTestRepititions*nofResponsesPerStimulus ] ;
        int [][] arrResponse = new int [ nofStimuli ][ nofTestRepititions*nofResponsesPerStimulus ];
        int [][] predictionHist = new int[ m_nofLearners ][ m_params.getNofCauses() ]; // use histograms to determine ZNeuron specializations
        int noWinnerCount = 0;

        int [] allZeroSpikeTrain = new int [ m_nofYNeurons ];
        
        for(int si=0; si<nofStimuli; si++){
            
            for(int rep=0; rep<nofTestRepititions; rep++){
                
                int repOffset = rep*nofResponsesPerStimulus;
                int [][] spikesY = m_encoder.encode(m_stimuli[si]);
                //int [][] temp = ModelUtils.pad2D(spikesY, 6, ModelUtils.MODE_PAD_COLS);

                for(int yt=0; yt<nofResponsesPerStimulus; yt++){

                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikesY[][]                    
                    int [] spikesYAtT = (yt < m_params.getEncDurationInMilSec())? ModelUtils.extract_columns(spikesY, yt) : allZeroSpikeTrain;

                    int responseIndex = repOffset+yt;
                    for(int zi=0; zi<m_nofLearners; zi++){

                        membranePotentialZ[zi][ si ][ responseIndex ] = m_arrZNeurons[zi].predict(spikesYAtT);
                    }
                    
                    int wtaResponse = m_wta.compete();
                    if(wtaResponse == AbstractCompetition.WTA_ALL){
                        
                        for(int zi=0; zi<m_nofLearners; zi++)
                            predictionHist[ zi ][ si ]++;
                    }
                    else if(wtaResponse != AbstractCompetition.WTA_NONE){
                        
                        predictionHist[ wtaResponse ][ si ]++;
                    }
                    else 
                        noWinnerCount++;
                    
                    arrResponse[ si ][ responseIndex ] = wtaResponse;
                }
            }
        }       

        ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response.csv", arrResponse);
        ModelPredictionTest.saveWeights(m_params.getMainOutputDir()+"weights.csv", m_arrZNeurons);    
        
        // print final prediction results
        System.out.println("final response: ");
        for(int si=0; si<nofStimuli; si++){
            
            System.out.println("s"+si+": ");
            for(int zi=0; zi<m_nofLearners; zi++){

                System.out.print("z"+zi+": ");
                System.out.print(membranePotentialZ[zi][ si ][ nofTestRepititions*nofResponsesPerStimulus-gapWidth-1 ]);
                System.out.println();
            }
        }
                
        int nofClasses = m_params.getNofCauses();
        System.out.println("test set size: "+m_params.getNofTestStimuli());
        
        // calc firing prob give cause and print counts
        double [][] firingProb = new double [ m_nofLearners ][ nofClasses ];
        double [] arrConditionalEntropy = new double [ m_nofLearners ];
        for(int zi=0; zi<m_nofLearners; zi++){

            System.out.print("z"+zi+": ");
            int sumPerLearner = 0;

            for(int i=0; i<nofClasses; i++){

                int count = predictionHist[ zi ][i];
                sumPerLearner += count;
                System.out.print(count + "  ");
            }
            for(int i=0; i<nofClasses; i++){

                firingProb[zi][i] = predictionHist[ zi ][i]/(double)sumPerLearner;
            }

            arrConditionalEntropy[ zi ] = ConditionalEntropy.calculate(firingProb[zi]);
            

            System.out.print("  \u03A3  " + sumPerLearner);
            System.out.print("  condEntr  " + arrConditionalEntropy[ zi ]);
            System.out.println();

        }
        System.out.println("No winner: "+noWinnerCount);
        
        ModelPredictionTest.savePredictionStats(m_params.getMainOutputDir()+"predictionStats.csv", firingProb, arrConditionalEntropy);

    }
    

    
    public void run(){
    
        System.out.println("running");
        learn();
        System.out.println("learn()...done");
        test();
        System.out.println("test()...done");
    }    
 
}
