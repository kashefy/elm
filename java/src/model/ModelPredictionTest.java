/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model;

/**
 *
 * @author woodstock
 */
//import com.sun.org.apache.bcel.internal.classfile.Code;
import model.features.OrientationMapTest;
import model.features.FeatureMapParams;
import model.features.OrientationMap;
import model.neuron.*;
import model.encoding.*;
import model.competition.*;
import model.utils.*;

import java.io.*;
import model.utils.files.AbstractDataLoader;
import model.utils.files.DataLoaderImageSetCSV;
import model.utils.files.FileIO;
import model.evaluation.*;

public class ModelPredictionTest {
    
    // test ZNeurons only
    public void test1(){
        
        int nofYNeurons = 2;
        int nofZNeurons = 2;
        int historyLengthZ = 2;
        int nofStimuli = 4;
        double [] initialWeights;
        int [][] spikes;
        ZNeuron [] arrZNeurons;
        arrZNeurons = new ZNeuron[ nofZNeurons ];
        LearnerParams learnerParams = new LearnerParams();
        learnerParams.setHistoryLength(historyLengthZ);
        learnerParams.setNofInputNodes(nofYNeurons);
          
        initialWeights = new double[ nofYNeurons ];
        for(int wi=0; wi<nofYNeurons; wi++){
            
            initialWeights[wi] = wi+1;
        }
        learnerParams.setWeightParams(true, 0.1, null);
        //learnerParams.setWeightParams(true, false, 0.1, initialWeights);
        
        for(int zi=0; zi<nofZNeurons; zi++){
            
            arrZNeurons[zi] = new ZNeuron();
            //arrZNeurons[zi].setHistoryLength(learnerHistoryLength);
            arrZNeurons[zi].init();
            //arrZNeurons[zi].setInitialweights(initialWeights);
            //arrZNeurons[zi].setRandomWeights(0.1);
        }  
        
        spikes = new int [ nofStimuli ][ nofYNeurons ];
        spikes[0] = new int []{0,0};
        spikes[1] = new int []{1,0};
        spikes[2] = new int []{0,1};
        spikes[3] = new int []{0,0};
        
        double [][] responseHistory = new double [ nofYNeurons ][ nofStimuli ];
        for(int t=0; t<nofStimuli; t++){
            
            for(int zi=0; zi<nofZNeurons; zi++){
                
                responseHistory[zi][t] = arrZNeurons[zi].predict(spikes[t]);
            }
        }
        
        for(int zi=0; zi<nofZNeurons; zi++){
            
            for(int t=0; t<nofStimuli; t++){
                
                System.out.print(responseHistory[zi][t]);
                System.out.print(' ');
            }
            System.out.println();
        }
    }
    

    
    // test ZNeurons with encoded spike trains and predefined weights
    public void test2(){
 
        // some parameters are reused in multiple paramter sections
        // stimuli parameters
        int nofRows     = 3;
        int nofCols     = 3;
        int [] stimuliDims = new int[]{nofRows,nofCols};
        int nofStimuli = 8;
        double [][] stimuli = new double [ nofStimuli ][ nofRows*nofCols ];
        
        // feature map parameters
        OrientationMap mapOrient = new OrientationMap(); 
        int supportRadius = 1;
        int nofScales = 1;
        double scalingFactor = 1.5;
        double orientationResolution = 30.0;
        double gaborFrequency = 1.0;
        double elongation = 0.65;
        
        // encoding parameters
        Encoder encoder = new Encoder();
        double frequencyEncoding    = 40;       // 40 Hz
        double deltaT               = 0.001;    // in sec => 1 ms
        int durationInMilSec        = 50;       // in ms durationOfSpikeTrain for each Y Neuron
        int popCodeFanOut = 1;
        int [][] spikesY;
        
        // prediction parameters
        int nofCauses = 6;
        int nofYNeurons;                // get value from enocder
        int nofZNeurons = nofCauses;
        int historyLengthZ = durationInMilSec;
        ZNeuron [] arrZNeurons = new ZNeuron[ nofZNeurons ];       
        double [] initialWeightsZ;
        double [][][] membranePotentialZ; // nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double learningRateEta = 0.01;
        LearnerParams learnerParams = new LearnerParams();

        // combined initializations
        
        // generate stimuli
        stimuli[0] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,90);
        stimuli[1] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,0);
        stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,30);
        stimuli[3] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,60);
        stimuli[4] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,120);
        stimuli[5] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,150);
        stimuli[6] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,45);
        stimuli[7] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,-45);

        mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,gaborFrequency,elongation);
        
        encoder.setFeatureMap(mapOrient);
        encoder.init(frequencyEncoding, deltaT, durationInMilSec, nofRows*nofCols*mapOrient.getNofFeatureSets(), popCodeFanOut);
        encoder.setInputDimensions(stimuliDims);
        spikesY = new int[ encoder.getNofEncoderNodes() ][ durationInMilSec ];
        
        // predictors/classifiers
        nofYNeurons = encoder.getNofEncoderNodes();
        learnerParams.setWeightParams(false,0,null);
        learnerParams.setHistoryLength(historyLengthZ);
        learnerParams.setNofInputNodes(nofYNeurons);
        learnerParams.setLearningRateEtaInitVal(learningRateEta);
        for(int zi=0; zi<nofZNeurons; zi++){
            
            arrZNeurons[zi] = new ZNeuron();
            //arrZNeurons[zi] = new ZNeuronCompact();
            arrZNeurons[zi].setParams(learnerParams);
            //arrZNeurons[zi].setHistoryLength(learnerHistoryLength);
            arrZNeurons[zi].init();
        } 
        // arbitrary weights for predictor Neurons :: Start
//        initialWeightsZ = new double[ nofYNeurons ];
//        System.arraycopy(stimuli[0], 0, initialWeightsZ, 0, nofRows*nofCols);
//        initialWeightsZ[nofRows*nofCols] = 1; // weight for bias term
//        arrZNeurons[0].setInitialweights(initialWeightsZ);
                
//        System.arraycopy(stimuli[1], 0, initialWeightsZ, 0, nofRows*nofCols);
//        initialWeightsZ[nofRows*nofCols] = 1; // weight for bias term
//        arrZNeurons[1].setInitialweights(initialWeightsZ);
        
        boolean setArbitraryWeights = true;
        if (setArbitraryWeights){
        // first element meant for bias term w0
        // 90 degrees vertical
        initialWeightsZ = new double[]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
        // already includes bias term
        arrZNeurons[0].setInitialweights(initialWeightsZ);
        
        // 0 degrees horizontal
        initialWeightsZ = new double[]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // already includes bias term
        arrZNeurons[1].setInitialweights(initialWeightsZ);
        
        //30 degrees/////////
        initialWeightsZ = new double[]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // already includes bias term
        arrZNeurons[2].setInitialweights(initialWeightsZ);   
        
        //60 degrees
        initialWeightsZ = new double[]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
        // already includes bias term
        arrZNeurons[3].setInitialweights(initialWeightsZ);    
        
        //120 degrees
        initialWeightsZ = new double[]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
        // already includes bias term
        arrZNeurons[4].setInitialweights(initialWeightsZ);   
        
        //150 degrees
        initialWeightsZ = new double[]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // already includes bias term
        arrZNeurons[5].setInitialweights(initialWeightsZ);   
        
        // arbitrary weights for predictor Neurons :: End
        }
        else{
            
            for(int zi=0; zi<nofZNeurons; zi++){
                
                arrZNeurons[ zi ].setRandomWeights(1.0); 
            }
        }

        int nofTimeSteps;
        //nofTimeSteps = 2;
        nofTimeSteps = nofStimuli;
        membranePotentialZ = new double [ nofZNeurons ][ nofStimuli ][ durationInMilSec ] ;
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            spikesY = encoder.encode(stimuli[stimuliIndex]);
            
            for(int yt=0; yt<durationInMilSec; yt++){
                
                // create array for spikes of all y neurons at time yt
                int [] spikesAtT = new int[ nofYNeurons ];

                for(int yi=0; yi<nofYNeurons; yi++){
                    spikesAtT[yi] = spikesY[yi][ yt ];
                    System.out.print(spikesAtT[yi]+" ");

                }
                System.out.println("");   
                
                for(int zi=0; zi<nofZNeurons; zi++){
     
                    membranePotentialZ[zi][ stimuliIndex ][yt] = arrZNeurons[zi].predict(spikesAtT);
                }
            }
        }
        
        // print weights
        System.out.println("weights:");
        for(int zi=0; zi<nofZNeurons; zi++){

            System.out.print("z"+zi+": ");
            double [] weights = arrZNeurons[zi].getWeights();
            for(int wi=0; wi<weights.length; wi++){

                System.out.print(weights[wi]+" ");
            }
            System.out.println();
        }        
 
        // print prediction results
        System.out.println("response: ");
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            System.out.println("s"+stimuliIndex+": ");
            for(int zi=0; zi<nofZNeurons; zi++){

                System.out.print("z"+zi+": ");
                for(int t=0; t<durationInMilSec; t++){

                    System.out.print(membranePotentialZ[zi][ stimuliIndex ][t]);
                    System.out.print(' ');
                }
                System.out.println();
            }
        }
        
    }//test2()
    
        // test ZNeurons with encoded spike trains by learning weights first
    public void test3(){
 
        // some parameters are reused in multiple paramter sections
        // stimuli parameters
        int nofRows     = 5;
        int nofCols     = 5;
        int [] stimuliDims = new int[]{nofRows,nofCols};
        int nofStimuli  = 8;
        double [][] stimuli = new double [ nofStimuli ][ nofRows*nofCols ];
        
        // feature map parameters
        OrientationMap mapOrient = new OrientationMap(); 
        int supportRadius = 2;
        int nofScales = 1;
        double scalingFactor = 1.5;
        double orientationResolution = 90.0;
        double gaborFrequency = 1.0;
        double elongation = 0.65;
        
        // encoding parameters
        Encoder encoder = new Encoder();
        double frequencyEncoding    = 40;       // 40 Hz
        double deltaT               = 0.001;    // in sec => 1 ms
        int durationInMilSec        = 50;       // in ms durationOfSpikeTrain for each Y Neuron
        int popCodeFanOut = 1;
        int [][] spikesY;
        
        // prediction parameters
        int nofCauses = 6;
        int nofYNeurons;                // get value from enocder
        int nofZNeurons = nofCauses;
        int historyLengthZ = durationInMilSec;
        ZNeuron [] arrZNeurons = new ZNeuron[ nofZNeurons ];       
        double [] initialWeightsZ;
        double [][][] membranePotentialZ; // nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double randomWeightScale = 0.01;
        double learningRateEta = 0.01;
        LearnerParams learnerParams = new LearnerParams();

        // combined initializations
        
        // generate stimuli
        stimuli[0] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,90);
        stimuli[1] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,0);
        stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,30);
        stimuli[3] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,60);
        stimuli[4] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,120);
        stimuli[5] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,150);
        stimuli[6] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,45);
        stimuli[7] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,-45);

        mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,gaborFrequency,elongation);
        
        encoder.setFeatureMap(mapOrient);
        encoder.init(frequencyEncoding, deltaT, durationInMilSec, nofRows*nofCols*mapOrient.getNofFeatureSets(), popCodeFanOut);
        encoder.setInputDimensions(stimuliDims);
        spikesY = new int[ encoder.getNofEncoderNodes() ][ durationInMilSec ];
        
        // predictors/classifiers
        nofYNeurons = encoder.getNofEncoderNodes();
        learnerParams.setWeightParams(true,randomWeightScale,null);
        learnerParams.setHistoryLength(historyLengthZ);
        learnerParams.setNofInputNodes(nofYNeurons);
        learnerParams.setLearningRateEtaInitVal(learningRateEta);
        for(int zi=0; zi<nofZNeurons; zi++){
            
            arrZNeurons[zi] = new ZNeuron();
            //arrZNeurons[zi] = new ZNeuronCompact();
            arrZNeurons[zi].setParams(learnerParams);
            arrZNeurons[zi].init();
//            arrZNeurons[zi].setRandomWeights(0.01); 
//            arrZNeurons[zi].setLearningRateEtaInitVal(0.1);
//            arrZNeurons[zi].letFire(true);
        } 
        
        // let each learn a different pattern
        // show each a single pattern during learning phase
        int nofRepititions = 1;
        for(int zi=0; zi<nofZNeurons; zi++){
            
            for(int rep=0; rep<nofRepititions; rep++){
                
                spikesY = encoder.encode(stimuli[zi]);
                double [] popCode = encoder.getPopCode();
                
                System.out.println("pop:");
                for(int yi=0; yi<nofYNeurons; yi++){
                    
                    System.out.print(popCode[ yi ] + " ");
                }
                
                for(int yt=0; yt<durationInMilSec; yt++){

                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikesY[][]
                    int [] spikesAtT = new int[ nofYNeurons ];

                    for(int yi=0; yi<nofYNeurons; yi++){

                        spikesAtT[yi] = spikesY[yi][ yt ];
                    }
                    
                    double pot = arrZNeurons[zi].predict(spikesAtT);
                    
                    System.out.println("pot z"+zi+": "+pot);
                    
                    double [] w;
                    
                    w = arrZNeurons[zi].getWeights();
                    System.out.print("prewz"+zi+": ");
                    for(int wi=0; wi<w.length; wi++)
                        System.out.print(w[wi]+" ");
                    
                    System.out.println();
                    
                    arrZNeurons[zi].update();
                    
                    w = arrZNeurons[zi].getWeights();
                    
                    System.out.print("postwz"+zi+": ");
                    for(int wi=0; wi<w.length; wi++)
                        System.out.print(w[wi]+" ");
                    
                    int abc = 123;
                }   
            }
        }
       
        // let's test
        
         int nofTimeSteps;
        //nofTimeSteps = 2;
        nofTimeSteps = nofStimuli;
        membranePotentialZ = new double [ nofZNeurons ][ nofStimuli ][ historyLengthZ ] ;
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            spikesY = encoder.encode(stimuli[stimuliIndex]);
            
            for(int yt=0; yt<durationInMilSec; yt++){
                
                // create array for spikes of all y neurons at time yt
                int [] spikesAtT = new int[ nofYNeurons ];

                for(int yi=0; yi<nofYNeurons; yi++){
                    spikesAtT[yi] = spikesY[yi][ yt ];
                    System.out.print(spikesAtT[yi]+" ");

                }
                System.out.println("");   
                
                for(int zi=0; zi<nofZNeurons; zi++){
     
                    membranePotentialZ[zi][ stimuliIndex ][yt] = arrZNeurons[zi].predict(spikesAtT);
                }
            }
        }       
        
        // print weights
        System.out.println("weights:");
        for(int zi=0; zi<nofZNeurons; zi++){

            System.out.print("z"+zi+": ");
            double [] weights = arrZNeurons[zi].getWeights();
            for(int wi=0; wi<weights.length; wi++){

                System.out.print(weights[wi]+" ");
            }
            System.out.println();
        }        
 
        // print prediction results
        System.out.println("response: ");
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            System.out.println("s"+stimuliIndex+": ");
            for(int zi=0; zi<nofZNeurons; zi++){

                System.out.print("z"+zi+": ");
                for(int t=0; t<historyLengthZ; t++){

                    System.out.print(membranePotentialZ[zi][ stimuliIndex ][t]);
                    System.out.print(' ');
                }
                System.out.println();
            }
        }
        
        // print final prediction results
        System.out.println("final response: ");
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            System.out.println("s"+stimuliIndex+": ");
            for(int zi=0; zi<nofZNeurons; zi++){

                System.out.print("z"+zi+": ");
                System.out.print(membranePotentialZ[zi][ stimuliIndex ][ historyLengthZ-1 ]);
                System.out.println();
            }
        }
        
    }// test3
    
    // include competition in primitive way, repeat training sample so that they'll specialize anyway
    public static void test4(){
        
        // some parameters are reused in multiple paramter sections
        // stimuli parameters
        int nofRows     = 11;
        int nofCols     = 11;
        int [] stimuliDims = new int[]{nofRows,nofCols};
        int nofStimuli  = 8;
        double [][] stimuli = new double [ nofStimuli ][ nofRows*nofCols ];
        nofStimuli = 6;
        
        // feature map parameters
        int supportRadius = 5;
        int nofScales = 1;
        double scalingFactor = 1.5;
        double orientationResolution = 30.0;
        double gaborFrequency = 1.0;
        double elongation = 0.65;
        
        FeatureMapParams featMapParams = new FeatureMapParams();
        
        featMapParams.setSupportRadius(supportRadius);
        featMapParams.setNofScales(nofScales);
        featMapParams.setScalingFactor(scalingFactor);
        featMapParams.setOrientation(orientationResolution);
        featMapParams.setGaborFrequency(gaborFrequency);
        featMapParams.setElongation(elongation);
        
        OrientationMap mapOrient = new OrientationMap(); 
        mapOrient.setParams(featMapParams);
        
        // encoding parameters
        Encoder encoder = new Encoder();
        double frequencyEncoding    = 2*40;       // 40 Hz
        double deltaT               = 0.001;    // in sec => 1 ms
        int durationInMilSec        = 50;       // in ms durationOfSpikeTrain for each Y Neuron
        int popCodeFanOut = 1;
        int [][] spikesY;
        int [][] spikesYWithGuardBand;
        
        // prediction parameters
        int nofCauses = 6;
        int nofYNeurons;                // get value from enocder
        int nofLearners = nofCauses;
        int learnerHistoryLength = 5;//durationInMilSec;
        ZNeuron [] arrZNeurons = new ZNeuron[ nofLearners ];       
        double [] initialWeightsZ;
        double [][][] membranePotentialZ; // nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double randomWeightScale = 0.01;
        double learningRateEta = 0.01;
        
        LearnerParams learnerParams = new LearnerParams();
        
        // competition parameters
                
        double competitionDt        = deltaT;
        double competitionMaxRate   = 1.0/(5.0*competitionDt);
        
        //// ou 
        double inhibitionAmplitude  = 1600;// overwritten later     // amplitude of inhibition (encoded rate * number of inputs);
        double inhibitionTau        = 0.005;    // Time constant of inhibition (gain of OU process)
        double inhibitonOffset      = 500;      // Offset of inhibition
    
        double ouTau    = 2.7e-3;   // Time constnt of OU process
        double ouSigma  = 2500;     // Variability of OU process
        double ouMu     = 150;      // Offset of OU process
        
        CompetitionParams competitionParams = new CompetitionParams();
        competitionParams.set_dt(competitionDt);
        competitionParams.setMaxRate(competitionMaxRate);
        competitionParams.setInhibitionParams(inhibitionAmplitude, inhibitionTau, inhibitonOffset);
        competitionParams.setOUParams(ouTau, ouSigma, ouMu);
        
        AbstractCompetition wta;
        //wta = new CompetitionWTAOU();
        wta = new WTAPoissonRate();
        wta.setParams(competitionParams);
        
        // combined initializations
        
        // generate stimuli
        stimuli[0] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,90);
        stimuli[1] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,0);
        stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,30);
        stimuli[3] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,60);
        stimuli[4] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,120);
        stimuli[5] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,150);
        stimuli[6] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,45);
        stimuli[7] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,-45);

        //mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,gaborFrequency,elongation);
        mapOrient.init();
        
        encoder.setFeatureMap(mapOrient);
        encoder.init(frequencyEncoding, deltaT, durationInMilSec, nofRows*nofCols*mapOrient.getNofFeatureSets(), popCodeFanOut);
        encoder.setInputDimensions(stimuliDims);
        //spikesY = new int[ encoder.getNofEncoderNodes() ][ durationInMilSec ];
        
        // predictors/classifiers
        nofYNeurons = encoder.getNofEncoderNodes();
        learnerParams.setWeightParams(true,randomWeightScale,null);
        learnerParams.setHistoryLength(learnerHistoryLength);
        learnerParams.setNofInputNodes(nofYNeurons);
        learnerParams.setLearningRateEtaInitVal(learningRateEta);
        for(int zi=0; zi<nofLearners; zi++){
            
            //arrZNeurons[zi] = new ZNeuron();
            arrZNeurons[zi] = new ZNeuronCompact();
            arrZNeurons[zi].setParams(learnerParams);
            arrZNeurons[zi].init();
            //arrZNeurons[zi].setHistoryLength(learnerHistoryLength);
            //arrZNeurons[zi].init(nofYNeurons);
            //arrZNeurons[zi].setRandomWeights(randomWeightScale); 
            //arrZNeurons[zi].setLearningRateEtaInitVal(learningRateEta);
        } 
        
        // wta initialization
        
//        if (wta instanceof CompetitionWTAOU) {
//            
//            CompetitionWTAOU wtaOU = (CompetitionWTAOU)wta;
//            inhibitionAmplitude = frequencyEncoding * nofYNeurons;
//            wtaOU.setInhibitionParams(inhibitionAmplitude, inhibitionTau, inhibitonOffset);
//            wtaOU.setOUParams(ouTau, ouSigma, ouMu);
//        }
//        else if(wta instanceof WTAPoissonRate){
//            
//            WTAPoissonRate wtaPR = (WTAPoissonRate)wta;
//            wtaPR.setParams(competitionDt, competitionMaxRate);
//        }
        wta.init();
        wta.refToLearners(arrZNeurons);
        
        // predict - compete - update
        int nofTrainRepititions = 100;
        int [][] arrResponse;
        int gapWidth = learnerHistoryLength;
        arrResponse = new int [ nofStimuli ][ nofTrainRepititions*(durationInMilSec+gapWidth) ];
        
        for(int stimulusIndex=0; stimulusIndex<nofStimuli; stimulusIndex++){
            
            int responseIndex = 0;
            for(int rep=0; rep<nofTrainRepititions; rep++){
                
                // transform input into set of spike trains
                spikesY = encoder.encode(stimuli[ stimulusIndex ]);
            
                // peek at population code for validation
//                double [] popCode = encoder.getPopCode();
//                System.out.println("s"+stimulusIndex+"-pop:");
//                for(int yi=0; yi<nofYNeurons; yi++){
//                    
//                    System.out.print(popCode[ yi ] + " ");
//                }
                
                // for each column in all spike trains
                // predict - compete - update
                for(int yt=0; yt<durationInMilSec+gapWidth; yt++){
                    
                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikesY[][]
                    int [] spikesYAtT;
                    if(yt < durationInMilSec){
                    
                        spikesYAtT = ModelUtils.extractColumns(spikesY, yt);
                    }
                    else
                        spikesYAtT = new int [ nofYNeurons ];
                    
                    // predict
                    for(int zi=0; zi<nofLearners; zi++){
                        
                        double membranePotential;
                        membranePotential = arrZNeurons[zi].predict(spikesYAtT);
//                        System.out.println("uz"+zi+": "+membranePotential);
//                        
//                        // peek at weights
//                        double [] w;
//                        w = arrZNeurons[zi].getWeights();
//                        System.out.print("prewz"+zi+": ");
//                        for(int wi=0; wi<w.length; wi++)
//                            System.out.print(w[wi]+" ");
//                        System.out.println();
                    }
                    
                    // let Z's compete before updating
                    int wtaResponse = wta.compete();
                    
                    arrResponse[ stimulusIndex ][ responseIndex++ ] = wtaResponse;
                    if(wtaResponse != AbstractCompetition.WTA_NONE){

                        int [] arrWTAResponse;
                        arrWTAResponse = wta.getOutcome();
                        for(int zi=0; zi<nofLearners; zi++){
                        
                            arrZNeurons[ zi ].letFire(arrWTAResponse[ zi ]==1);
                            //    arrZNeurons[ zi ].letFire(false);
                        }
                    }
                    //System.out.println("wta "+ wtaResponse);

                    // update
                    for(int zi=0; zi<nofLearners; zi++){
                        
                        arrZNeurons[zi].update();
                        
                        // peek at new weights
//                        double [] w = arrZNeurons[zi].getWeights();            
//                        System.out.print("postwz"+zi+": ");
//                        for(int wi=0; wi<w.length; wi++)
//                            System.out.print(w[wi]+" ");
//                        System.out.println();
                    }
                }  
            }    
        }
        
        try{

            // Create file  
            PrintWriter pw = new PrintWriter(new FileWriter(".\\data\\output\\responseTraining.csv"));

            for(int r=0; r<nofStimuli; r++){

                for(int c=0; c<nofTrainRepititions*durationInMilSec; c++){

                    pw.print(arrResponse[r][c]);
                    if(c<nofTrainRepititions*durationInMilSec-1)
                        pw.print(",");
                }
                pw.println();
            }
            pw.close();     //Close the output stream
         }
         catch (Exception e){//Catch exception if any
                  
                System.err.println("Error: " + e.getMessage());
         }

        // let's test

        int nofTimeSteps;
        //nofTimeSteps = 2;
        nofTimeSteps = nofStimuli;
        int nofTestRepititions = 1000;
        membranePotentialZ = new double [ nofLearners ][ nofStimuli ][ durationInMilSec ] ;
        arrResponse = new int [ nofTimeSteps ][ nofTestRepititions*(durationInMilSec) ];
        
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            int responseIndex = 0;
            for(int rep=0; rep<nofTestRepititions; rep++){
                
                spikesY = encoder.encode(stimuli[stimuliIndex]);
                //int [][] temp = ModelUtils.pad2D(spikesY, 6, ModelUtils.MODE_PAD_COLS);

                for(int yt=0; yt<durationInMilSec; yt++){

                    // create array for spikes of all y neurons at time yt
                    int [] spikesYAtT;
                    
                    spikesYAtT = ModelUtils.extractColumns(spikesY, yt);

//                    for(int yi=0; yi<nofYNeurons; yi++){
//
//                        System.out.print(spikesYAtT[yi]+" ");
//
//                    }
//                    System.out.println("");   

                    for(int zi=0; zi<nofLearners; zi++){

                        membranePotentialZ[zi][ stimuliIndex ][yt] = arrZNeurons[zi].predict(spikesYAtT);
                    }
                    wta.refToLearners(arrZNeurons);
                    int wtaResponse = wta.compete();
                    

                    //System.out.println("wta "+ wtaResponse);
                    
                    arrResponse[ stimuliIndex ][ responseIndex++ ] = wtaResponse;
                }
            }
        }       
        
         try{

            // Create file  
            PrintWriter pw = new PrintWriter(new FileWriter(".\\data\\output\\response.csv"));

            for(int r=0; r<nofStimuli; r++){

                for(int c=0; c<nofTestRepititions*durationInMilSec; c++){

                    pw.print(arrResponse[r][c]);
                    if(c<nofTestRepititions*durationInMilSec-1)
                        pw.print(",");
                }
                pw.println();
            }
            pw.close();     //Close the output stream
         }
         catch (Exception e){//Catch exception if any
                  
                System.err.println("Error: " + e.getMessage());
         }
         System.out.print("done");
        
        // print weights
//        System.out.println("weights:");
//        for(int zi=0; zi<nofZNeurons; zi++){
//
//            System.out.print("z"+zi+": ");
//            double [] weights = arrZNeurons[zi].getWeights();
//            for(int wi=0; wi<weights.length; wi++){
//
//                System.out.print(weights[wi]+" ");
//            }
//            System.out.println();
//        }        
 
        // print prediction results
//        System.out.println("response: ");
//        for(int stimulusIndex=0; stimulusIndex<nofTimeSteps; stimulusIndex++){
//            
//            System.out.println("s"+stimulusIndex+": ");
//            for(int zi=0; zi<nofZNeurons; zi++){
//
//                System.out.print("z"+zi+": ");
//                for(int t=0; t<historyLengthZ; t++){
//
//                    System.out.print(membranePotentialZ[zi][ stimulusIndex ][t]);
//                    System.out.print(' ');
//                }
//                System.out.println();
//            }
//        }
        
        // print final prediction results
        System.out.println("final response: ");
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            System.out.println("s"+stimuliIndex+": ");
            for(int zi=0; zi<nofLearners; zi++){

                System.out.print("z"+zi+": ");
                System.out.print(membranePotentialZ[zi][ stimuliIndex ][ learnerHistoryLength-1 ]);
                System.out.println();
            }
        }       
        String strWeightFile = ".\\data\\output\\weightsBarSet.csv";
        saveWeights(strWeightFile, arrZNeurons);
    }// test4()
    
    // test4() wrapped around a timer. see test4()
    public static void test4Timed(){
        
        Stopwatch timer;
        timer = new Stopwatch();
        timer.start();
        
        test4();
        
        timer.stop();
        System.out.println(timer.getElapsedTime());
        //System.out.println(timer.toString());
        
    }// test4Timed()
    
    // compare ZNeuron with ZNeuronCompact
    public static void test5(){

        // some parameters are reused in multiple paramter sections
        // stimuli parameters
        int nofRows     = 11;
        int nofCols     = 11;
        int [] stimuliDims = new int[]{nofRows,nofCols};
        int nofStimuli  = 8;
        double [][] stimuli = new double [ nofStimuli ][ nofRows*nofCols ];
        nofStimuli = 6;
        
        // feature map parameters
        OrientationMap mapOrient = new OrientationMap(); 
        int supportRadius = 5;
        int nofScales = 1;
        double scalingFactor = 1.5;
        double orientationResolution = 30.0;
        double gaborFrequency = 1.0;
        double elongation = 0.65;
        
        // encoding parameters
        Encoder encoder = new Encoder();
        double frequencyEncoding    = 2*40;       // 40 Hz
        double deltaT               = 0.001;    // in sec => 1 ms
        int durationInMilSec        = 50;       // in ms durationOfSpikeTrain for each Y Neuron
        int popCodeFanOut = 1;
        int [][] spikesY;
        int [][] spikesYWithGuardBand;
        
        // prediction parameters
        int nofCauses = 6;
        int nofYNeurons;                // get value from enocder
        int nofZNeurons = nofCauses;
        int historyLengthZ = 5;//durationInMilSec;
        ZNeuron [] arrZNeurons = new ZNeuron[ nofZNeurons ]; 
        ZNeuron [] arrZNeuronsC = new ZNeuron[ nofZNeurons ];
        double [] initialWeightsZ;
        double [][][] membranePotentialZ; // nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double randomWeightScale = 0.01;
        double learningRateEta = 0.01;
        
        LearnerParams learnerParams = new LearnerParams();
        
        // competition parameters
                
        double competitionDt        = deltaT;
        double competitionMaxRate   = 1.0/(5.0*competitionDt);
        
        //// ou 
        double inhibitionAmplitude  = 1600;// overwritten later     // amplitude of inhibition (encoded rate * number of inputs);
        double inhibitionTau        = 0.005;    // Time constant of inhibition (gain of OU process)
        double inhibitonOffset      = 500;      // Offset of inhibition
    
        double ouTau    = 2.7e-3;   // Time constnt of OU process
        double ouSigma  = 2500;     // Variability of OU process
        double ouMu     = 150;      // Offset of OU process
        
        CompetitionParams competitionParams = new CompetitionParams();
        competitionParams.set_dt(competitionDt);
        competitionParams.setMaxRate(competitionMaxRate);
        competitionParams.setInhibitionParams(inhibitionAmplitude, inhibitionTau, inhibitonOffset);
        competitionParams.setOUParams(ouTau, ouSigma, ouMu);
        
        AbstractCompetition wta;
        //wta = new CompetitionWTAOU();
        wta = new WTAPoissonRate();
        wta.setParams(competitionParams);
        
        // combined initializations
        
        // generate stimuli
        stimuli[0] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,90);
        stimuli[1] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,0);
        stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,30);
        stimuli[3] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,60);
        stimuli[4] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,120);
        stimuli[5] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,150);
        stimuli[6] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,45);
        stimuli[7] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,-45);

        mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,gaborFrequency,elongation);
        
        encoder.setFeatureMap(mapOrient);
        encoder.init(frequencyEncoding, deltaT, durationInMilSec, nofRows*nofCols*mapOrient.getNofFeatureSets(), popCodeFanOut);
        encoder.setInputDimensions(stimuliDims);
        //spikesY = new int[ encoder.getNofEncoderNodes() ][ durationInMilSec ];
        
        // predictors/classifiers
        nofYNeurons = encoder.getNofEncoderNodes();
        learnerParams.setWeightParams(true,randomWeightScale,null);
        learnerParams.setHistoryLength(historyLengthZ);
        learnerParams.setNofInputNodes(nofYNeurons);
        learnerParams.setLearningRateEtaInitVal(learningRateEta);
        for(int zi=0; zi<nofZNeurons; zi++){
            
            arrZNeurons[zi] = new ZNeuron();
            //arrZNeurons[zi] = new ZNeuronCompact();
            arrZNeurons[zi].setParams(learnerParams);
            arrZNeurons[zi].init();
            //arrZNeurons[zi].setHistoryLength(learnerHistoryLength);
            //arrZNeurons[zi].init(nofYNeurons);
            //arrZNeurons[zi].setRandomWeights(randomWeightScale); 
            //arrZNeurons[zi].setLearningRateEtaInitVal(learningRateEta);
            arrZNeuronsC[zi] = new ZNeuronCompact();
            arrZNeuronsC[zi].setParams(learnerParams);
            
            arrZNeuronsC[zi].init();
            arrZNeuronsC[zi].setInitialweights(arrZNeurons[zi].getWeights());
        } 
        
        // making sure weights match
        for(int zi=0; zi<nofZNeurons; zi++){
            
            double [] zw = arrZNeurons[zi].getWeights();
            double [] zcw = arrZNeuronsC[zi].getWeights();
            
            int nofZWeights = zw.length;
            int nofZCWeights = zcw.length;
            
            if(nofZWeights != nofZCWeights)
                System.err.println("AAA");
            
            for(int wi=0; wi<nofZWeights; wi++){
                
                if(zw[wi] != zcw[wi])
                    System.err.println("BBB");
            }
        }
        
        // wta initialization
        
//        if (wta instanceof CompetitionWTAOU) {
//            
//            CompetitionWTAOU wtaOU = (CompetitionWTAOU)wta;
//            inhibitionAmplitude = frequencyEncoding * nofYNeurons;
//            wtaOU.setInhibitionParams(inhibitionAmplitude, inhibitionTau, inhibitonOffset);
//            wtaOU.setOUParams(ouTau, ouSigma, ouMu);
//        }
//        else if(wta instanceof WTAPoissonRate){
//            
//            WTAPoissonRate wtaPR = (WTAPoissonRate)wta;
//            wtaPR.setParams(competitionDt, competitionMaxRate);
//        }
        wta.init();
        wta.refToLearners(arrZNeurons);
        
        // predict - compete - update
        int nofTrainRepititions = 100;
        int [][] arrResponse;
        int gapWidth = historyLengthZ;
        arrResponse = new int [nofStimuli][ nofTrainRepititions*(durationInMilSec+gapWidth) ];
        
        for(int stimulusIndex=0; stimulusIndex<nofStimuli; stimulusIndex++){
            
            int responseIndex = 0;
            for(int rep=0; rep<nofTrainRepititions; rep++){
                
                // transform input into set of spike trains
                spikesY = encoder.encode(stimuli[ stimulusIndex ]);
            
                // peek at population code for validation
//                double [] popCode = encoder.getPopCode();
//                System.out.println("s"+stimulusIndex+"-pop:");
//                for(int yi=0; yi<nofYNeurons; yi++){
//                    
//                    System.out.print(popCode[ yi ] + " ");
//                }
                
                // for each column in all spike trains
                // predict - compete - update
                for(int yt=0; yt<durationInMilSec+gapWidth; yt++){
                    
                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikesY[][]
                    int [] spikesYAtT;
                    if(yt < durationInMilSec){
                    
                        spikesYAtT = ModelUtils.extractColumns(spikesY, yt);
                    }
                    else
                        spikesYAtT = new int [ nofYNeurons ];
                    
                    // predict
                    for(int zi=0; zi<nofZNeurons; zi++){
                        
                        double membranePotential;
                        membranePotential = arrZNeurons[zi].predict(spikesYAtT);
                        
                        double membranePotentialC;
                        membranePotentialC = arrZNeuronsC[zi].predict(spikesYAtT);
                        
                        if(membranePotential != membranePotentialC)
                            System.err.println("MMM");
                        
//                        System.out.println("uz"+zi+": "+membranePotential);
//                        
//                        // peek at weights
//                        double [] w;
//                        w = arrZNeurons[zi].getWeights();
//                        System.out.print("prewz"+zi+": ");
//                        for(int wi=0; wi<w.length; wi++)
//                            System.out.print(w[wi]+" ");
//                        System.out.println();
                    }
                    
                    // let Z's compete before updating
                    int wtaResponse = wta.compete();
                    
                    arrResponse[ stimulusIndex ][ responseIndex++ ] = wtaResponse;
                    if(wtaResponse != AbstractCompetition.WTA_NONE){

                        int [] arrWTAResponse;
                        arrWTAResponse = wta.getOutcome();
                        for(int zi=0; zi<nofZNeurons; zi++){
                        
                            arrZNeurons[ zi ].letFire(arrWTAResponse[ zi ]==1);
                            //    arrZNeurons[ zi ].letFire(false);
                            arrZNeuronsC[ zi ].letFire(arrWTAResponse[ zi ]==1);
                        }
                    }
                    //System.out.println("wta "+ wtaResponse);

                    // update
                    for(int zi=0; zi<nofZNeurons; zi++){
                        
                        arrZNeurons[zi].update();
                        
                        // peek at new weights
//                        double [] w = arrZNeurons[zi].getWeights();            
//                        System.out.print("postwz"+zi+": ");
//                        for(int wi=0; wi<w.length; wi++)
//                            System.out.print(w[wi]+" ");
//                        System.out.println();
                        arrZNeuronsC[zi].update();
                        
                                    
                        double [] zw = arrZNeurons[zi].getWeights();
                        double [] zcw = arrZNeuronsC[zi].getWeights();

                        int nofZWeights = zw.length;
                        int nofZCWeights = zcw.length;

                        if(nofZWeights != nofZCWeights)
                            System.err.println("AA2");

                        for(int wi=0; wi<nofZWeights; wi++){

                            if(zw[wi] != zcw[wi])
                                System.err.println("BB2");
                        }
                    }
                }  
            }    
        }
        
        try{

            // Create file  
            PrintWriter pw = new PrintWriter(new FileWriter(".\\data\\output\\responseTraining.csv"));

            for(int r=0; r<nofStimuli; r++){

                for(int c=0; c<nofTrainRepititions*durationInMilSec; c++){

                    pw.print(arrResponse[r][c]);
                    if(c<nofTrainRepititions*durationInMilSec-1)
                        pw.print(",");
                }
                pw.println();
            }
            pw.close();     //Close the output stream
         }
         catch (Exception e){//Catch exception if any
                  
                System.err.println("Error: " + e.getMessage());
         }

        // let's test

        int nofTimeSteps;
        //nofTimeSteps = 2;
        nofTimeSteps = nofStimuli;
        int nofTestRepititions = 1000;
        membranePotentialZ = new double [ nofZNeurons ][ nofStimuli ][ durationInMilSec ] ;
        arrResponse = new int [ nofTimeSteps ][ nofTestRepititions*(durationInMilSec) ];
        
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            int responseIndex = 0;
            for(int rep=0; rep<nofTestRepititions; rep++){
                
                spikesY = encoder.encode(stimuli[stimuliIndex]);
                //int [][] temp = ModelUtils.pad2D(spikesY, 6, ModelUtils.MODE_PAD_COLS);

                for(int yt=0; yt<durationInMilSec; yt++){

                    // create array for spikes of all y neurons at time yt
                    int [] spikesYAtT;
                    
                    spikesYAtT = ModelUtils.extractColumns(spikesY, yt);

//                    for(int yi=0; yi<nofYNeurons; yi++){
//
//                        System.out.print(spikesYAtT[yi]+" ");
//
//                    }
//                    System.out.println("");   

                    for(int zi=0; zi<nofZNeurons; zi++){

                        membranePotentialZ[zi][ stimuliIndex ][yt] = arrZNeurons[zi].predict(spikesYAtT);
                        double mC = arrZNeuronsC[zi].predict(spikesYAtT);
                        
                        if(mC != membranePotentialZ[zi][ stimuliIndex ][yt])
                            System.err.println("TSM");
                    }
                    wta.refToLearners(arrZNeurons);
                    int wtaResponse = wta.compete();
                    

                    //System.out.println("wta "+ wtaResponse);
                    
                    arrResponse[ stimuliIndex ][ responseIndex++ ] = wtaResponse;
                }
            }
        }       
        
         try{

            // Create file  
            PrintWriter pw = new PrintWriter(new FileWriter(".\\data\\output\\response.csv"));

            for(int r=0; r<nofStimuli; r++){

                for(int c=0; c<nofTestRepititions*durationInMilSec; c++){

                    pw.print(arrResponse[r][c]);
                    if(c<nofTestRepititions*durationInMilSec-1)
                        pw.print(",");
                }
                pw.println();
            }
            pw.close();     //Close the output stream
         }
         catch (Exception e){//Catch exception if any
                  
                System.err.println("Error: " + e.getMessage());
         }
         System.out.print("done");
        
        // print weights
//        System.out.println("weights:");
//        for(int zi=0; zi<nofZNeurons; zi++){
//
//            System.out.print("z"+zi+": ");
//            double [] weights = arrZNeurons[zi].getWeights();
//            for(int wi=0; wi<weights.length; wi++){
//
//                System.out.print(weights[wi]+" ");
//            }
//            System.out.println();
//        }        
 
        // print prediction results
//        System.out.println("response: ");
//        for(int stimulusIndex=0; stimulusIndex<nofTimeSteps; stimulusIndex++){
//            
//            System.out.println("s"+stimulusIndex+": ");
//            for(int zi=0; zi<nofZNeurons; zi++){
//
//                System.out.print("z"+zi+": ");
//                for(int t=0; t<historyLengthZ; t++){
//
//                    System.out.print(membranePotentialZ[zi][ stimulusIndex ][t]);
//                    System.out.print(' ');
//                }
//                System.out.println();
//            }
//        }
        
        // print final prediction results
        System.out.println("final response: ");
        for(int stimuliIndex=0; stimuliIndex<nofTimeSteps; stimuliIndex++){
            
            System.out.println("s"+stimuliIndex+": ");
            for(int zi=0; zi<nofZNeurons; zi++){

                System.out.print("z"+zi+": ");
                System.out.print(membranePotentialZ[zi][ stimuliIndex ][ historyLengthZ-1 ]);
                System.out.println();
            }
        }       
        String strWeightFile = ".\\data\\output\\weightsBarSet.csv";
        saveWeights(strWeightFile, arrZNeurons);
    }

    // simulate using small portion from MNIST data
    public void testOnMNIST(){
        
        //// some parameters are reused in multiple sections
        // stimuli parameters
        AbstractDataLoader dataLoader;
        String strDataDir = ".\\data\\input\\MNIST";
        
        dataLoader = new DataLoaderImageSetCSV();
        dataLoader.setParams(strDataDir);
                
        int nofRows;
        int nofCols;
        int [] stimuliDims;
        int nofStimuli;
        
        // feature map parameters
        OrientationMap mapOrient = new OrientationMap(); 
        int supportRadius       = 14;
        int nofScales           = 1;
        double scalingFactor    = 1.5;
        double orientationResolution = 30.0;
        double gaborFrequency   = 1.0;
        double elongation       = 0.65;
        
        // encoding parameters
        Encoder encoder = new Encoder();
        double frequencyEncoding    = 2*40;       // 40 Hz
        double deltaT               = 0.001;    // in sec => 1 ms
        int durationInMilSec        = 50;       // in ms durationOfSpikeTrain for each Y Neuron
        int popCodeFanOut           = 1;
        int [][] spikesY;
        int [][] spikesYWithGuardBand;
        
        // prediction parameters
        int nofCauses       = 6;
        int nofYNeurons;                // get value from enocder
        int nofZNeurons     = nofCauses;
        int historyLengthZ  = 5;//durationInMilSec;
        ZNeuron [] arrZNeurons = new ZNeuron[ nofZNeurons ];       
        double [] initialWeightsZ;
        double [][][] membranePotentialZ; // nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double randomWeightScale = 0.01;
        double learningRateEta = 0.01;
        LearnerParams learnerParams = new LearnerParams();
        
        // competition parameters     
        
        //-poisson rate
        double competitionDt        = deltaT;
        double competitionMaxRate   = 1.0/(5.0*competitionDt);
        
        //-ou 
        double inhibitionAmplitude  = 1600;// overwritten later     // amplitude of inhibition (encoded rate * number of inputs);
        double inhibitionTau        = 0.005;    // Time constant of inhibition (gain of OU process)
        double inhibitonOffset      = 500;      // Offset of inhibition
    
        double ouTau    = 2.7e-3;   // Time constnt of OU process
        double ouSigma  = 2500;     // Variability of OU process
        double ouMu     = 150;      // Offset of OU process
        
        CompetitionParams competitionParams = new CompetitionParams();
        competitionParams.set_dt(competitionDt);
        competitionParams.setMaxRate(competitionMaxRate);
        competitionParams.setInhibitionParams(inhibitionAmplitude, inhibitionTau, inhibitonOffset);
        competitionParams.setOUParams(ouTau, ouSigma, ouMu);
        
        AbstractCompetition wta;
        //wta = new CompetitionWTAOU();
        wta = new WTAPoissonRate();
        wta.setParams(competitionParams);
        
        //// combined, interdependent initializations
        System.out.println("initializing");
        dataLoader.init();
        dataLoader.load();
        
        nofStimuli  = dataLoader.getNofSamples();
        stimuliDims = dataLoader.getDims();
        nofRows     = stimuliDims[ FileIO.DIM_INDEX_ROWS ];
        nofCols     = stimuliDims[ FileIO.DIM_INDEX_COLS ];
         
        mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,gaborFrequency,elongation);
        
        encoder.setFeatureMap(mapOrient);
        encoder.init(frequencyEncoding, deltaT, durationInMilSec, nofRows*nofCols*mapOrient.getNofFeatureSets(), popCodeFanOut);
        encoder.setInputDimensions(stimuliDims);
        //spikesY = new int[ encoder.getNofEncoderNodes() ][ durationInMilSec ];
        
        
        // predictors/classifiers
        nofYNeurons = encoder.getNofEncoderNodes();
        learnerParams.setWeightParams(true,randomWeightScale,null);
        learnerParams.setHistoryLength(historyLengthZ);
        learnerParams.setNofInputNodes(nofYNeurons);
        learnerParams.setLearningRateEtaInitVal(learningRateEta);
        for(int zi=0; zi<nofZNeurons; zi++){
            
            //arrZNeurons[zi] = new ZNeuron();
            arrZNeurons[zi] = new ZNeuronCompact();
            arrZNeurons[zi].setParams(learnerParams);
            arrZNeurons[zi].init();
//            arrZNeurons[zi].setRandomWeights(0.01); 
//            arrZNeurons[zi].setLearningRateEtaInitVal(0.01);
        } 
        
        // wta initialization
        wta.init();
        wta.refToLearners(arrZNeurons);
        
        //// predict - compete - update
        System.out.println("training");
        
        int trainIndexStart = 0;
        int trainIndexEnd = (int)Math.floor(nofStimuli*0.75);
        int nofTrainRepititions = 1;
        int [][] arrResponse;
        int gapWidth = historyLengthZ;
        arrResponse = new int [ nofStimuli ][ nofTrainRepititions*(durationInMilSec+gapWidth) ];
        //ActivityMask[] activity = new ActivityMask[ ]
        
        for(int stimulusIndex=trainIndexStart; stimulusIndex<trainIndexEnd; stimulusIndex++){
            
            int responseIndex = 0;
            for(int rep=0; rep<nofTrainRepititions; rep++){
                
                // transform input into set of spike trains
                double [] stimulus = dataLoader.getSample(stimulusIndex);
                spikesY = encoder.encode(stimulus);
            
                // peek at population code for validation
//                double [] popCode = encoder.getPopCode();
//                System.out.println("s"+stimulusIndex+"-pop:");
//                for(int yi=0; yi<nofYNeurons; yi++){
//                    
//                    System.out.print(popCode[ yi ] + " ");
//                }
                
                // for each column in all spike trains
                // predict - compete - update
                for(int yt=0; yt<durationInMilSec+gapWidth; yt++){
                    
                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikesY[][]
                    int [] spikesYAtT;
                    if(yt < durationInMilSec){
                    
                        spikesYAtT = ModelUtils.extractColumns(spikesY, yt);
                    }
                    else
                        spikesYAtT = new int [ nofYNeurons ];
                    
                    // predict
                    for(int zi=0; zi<nofZNeurons; zi++){
                        
                        double membranePotential;
                        membranePotential = arrZNeurons[zi].predict(spikesYAtT);
//                        System.out.println("uz"+zi+": "+membranePotential);
//                        
//                        // peek at weights
//                        double [] w;
//                        w = arrZNeurons[zi].getWeights();
//                        System.out.print("prewz"+zi+": ");
//                        for(int wi=0; wi<w.length; wi++)
//                            System.out.print(w[wi]+" ");
//                        System.out.println();
                    }
                    
                    // let Z's compete before updating
                    int wtaResponse = wta.compete();
                    
                    arrResponse[ stimulusIndex ][ responseIndex++ ] = wtaResponse;
                    if(wtaResponse != AbstractCompetition.WTA_NONE){

                        int [] arrWTAResponse;
                        arrWTAResponse = wta.getOutcome();
                        for(int zi=0; zi<nofZNeurons; zi++){
                        
                            arrZNeurons[ zi ].letFire(arrWTAResponse[ zi ]==1);
                            //    arrZNeurons[ zi ].letFire(false);
                        }
                    }
                    //System.out.println("wta "+ wtaResponse);

                    // update
                    for(int zi=0; zi<nofZNeurons; zi++){
                        
                        arrZNeurons[zi].update();
                        
                        // peek at new weights
//                        double [] w = arrZNeurons[zi].getWeights();            
//                        System.out.print("postwz"+zi+": ");
//                        for(int wi=0; wi<w.length; wi++)
//                            System.out.print(w[wi]+" ");
//                        System.out.println();
                    }
                }  
            }    
        }
        
        try{

            // Create file  
            PrintWriter pw = new PrintWriter(new FileWriter(".\\data\\output\\responseTraining.csv"));

            for(int r=0; r<nofStimuli; r++){

                for(int c=0; c<nofTrainRepititions*durationInMilSec; c++){

                    pw.print(arrResponse[r][c]);
                    if(c<nofTrainRepititions*durationInMilSec-1)
                        pw.print(",");
                }
                pw.println();
            }
            pw.close();     //Close the output stream
         }
         catch (Exception e){//Catch exception if any
                  
                System.err.println("Error: " + e.getMessage());
         }

        // let's test and evaluate using an independent test set
        System.out.println("testing");

        int testIndexStart = trainIndexEnd;
        int testIndexEnd;
        testIndexEnd = nofStimuli;
        int nofTestStimuli = testIndexEnd-testIndexStart+1;
        int nofTestRepititions = 1;
        membranePotentialZ = new double [ nofZNeurons ][ nofTestStimuli ][ durationInMilSec ] ;
        arrResponse = new int [ nofTestStimuli ][ nofTestRepititions*(durationInMilSec) ];
        int noWinnerCount = 0;
        
        // determine which label a ZNeuron is responding to
        int nofClasses = dataLoader.getNofClasses();
        
        int [] arrLabelNames = new int [ nofClasses ];
        for(int ci=0; ci<nofClasses; ci++)
            arrLabelNames[ ci ] = -1;
        
        int nofClassesFound = 0;
        int [] arrLabels = new int [ nofTestStimuli ];
        for(int li=0; li<nofTestStimuli; li++){
            
            int labelTemp = dataLoader.getLabel(li);
            arrLabels[ li ] = labelTemp;
            
            if(nofClassesFound==0){
                
                // first entry
                arrLabelNames[ nofClassesFound++ ] = labelTemp;
            }
            else if(nofClassesFound<nofClasses){
                
                int backTracki = nofClassesFound-1;
                boolean alreadyListed = false;
                while(backTracki>=0 && !alreadyListed){
                    
                    int listedLabel = arrLabelNames[backTracki];
                    alreadyListed = labelTemp == listedLabel;
                    backTracki--;
                }
                if(!alreadyListed){
                    
                    // add as new listing
                    arrLabelNames[ nofClassesFound++ ] = labelTemp;
                }
            }
        }
        if(nofClassesFound < nofClasses){
            System.err.print("Not all classes represented in test set.");
        }
        // use histograms to determine ZNeuron specializations
        int [][] predictionHist = new int[ nofZNeurons ][ nofClasses ]; 
        
        int relStimulusIndex = 0;
        for(int stimulusIndex=testIndexStart; stimulusIndex<testIndexEnd; stimulusIndex++){
            
            int responseIndex = 0;
            for(int rep=0; rep<nofTestRepititions; rep++){
                
                double [] stimulus = dataLoader.getSample(stimulusIndex);
                spikesY = encoder.encode(stimulus);
                //int [][] temp = ModelUtils.pad2D(spikesY, 6, ModelUtils.MODE_PAD_COLS);

                for(int yt=0; yt<durationInMilSec; yt++){

                    // create array for spikes of all y neurons at time yt
                    int [] spikesYAtT;
                    
                    spikesYAtT = ModelUtils.extractColumns(spikesY, yt);

//                    for(int yi=0; yi<nofYNeurons; yi++){
//
//                        System.out.print(spikesYAtT[yi]+" ");
//
//                    }
//                    System.out.println("");   

                    for(int zi=0; zi<nofZNeurons; zi++){

                        membranePotentialZ[zi][ relStimulusIndex ][yt] = arrZNeurons[zi].predict(spikesYAtT);
                    }
                    wta.refToLearners(arrZNeurons);
                    int wtaResponse = wta.compete();
                    
                    if(wtaResponse != AbstractCompetition.WTA_NONE){
                        
                        for(int li=0; li<nofClasses; li++){

                            if(arrLabels[ relStimulusIndex ] == arrLabelNames[ li ])
                                predictionHist[ wtaResponse ][ li ]++;
                        }
                    }
                    else
                        noWinnerCount++;

                    //System.out.println("wta "+ wtaResponse);
                    
                    arrResponse[ relStimulusIndex ][ responseIndex++ ] = wtaResponse;
                }
            }
            relStimulusIndex++;
        }      
        
         try{

            // Create file  
            PrintWriter pw = new PrintWriter(new FileWriter(".\\data\\output\\response.csv"));

            for(int r=0; r<nofTestStimuli; r++){

                for(int c=0; c<nofTestRepititions*durationInMilSec; c++){

                    pw.print(arrResponse[r][c]);
                    if(c<nofTestRepititions*durationInMilSec-1)
                        pw.print(",");
                }
                pw.println();
            }
            pw.close();     //Close the output stream
         }
         catch (Exception e){//Catch exception if any
                  
                System.err.println("Error: " + e.getMessage());
         }
         System.out.println("done");
        
        // print weights
//        System.out.println("weights:");
//        for(int zi=0; zi<nofZNeurons; zi++){
//
//            System.out.print("z"+zi+": ");
//            double [] weights = arrZNeurons[zi].getWeights();
//            for(int wi=0; wi<weights.length; wi++){
//
//                System.out.print(weights[wi]+" ");
//            }
//            System.out.println();
//        }        
 
        // print prediction results
//        System.out.println("response: ");
//        for(int stimulusIndex=0; stimulusIndex<nofTimeSteps; stimulusIndex++){
//            
//            System.out.println("s"+stimulusIndex+": ");
//            for(int zi=0; zi<nofZNeurons; zi++){
//
//                System.out.print("z"+zi+": ");
//                for(int t=0; t<historyLengthZ; t++){
//
//                    System.out.print(membranePotentialZ[zi][ stimulusIndex ][t]);
//                    System.out.print(' ');
//                }
//                System.out.println();
//            }
//        }
            
        
        
         // print prediction histogram
         // print header
        
         System.out.print("test set size: "+nofTestStimuli);
         System.out.print("labelNames: ");
        
         for(int i=0; i<nofClasses; i++){
   
             System.out.print(" "+arrLabelNames[i]);       
        
         }
         System.out.println();
        
         // print count
         double [][] firingProb = new double [ nofZNeurons ][ nofClasses ];
         double [] arrConditionalEntropy = new double [ nofZNeurons ];
         for(int zi=0; zi<nofZNeurons; zi++){

            System.out.print("z"+zi+": ");
            int sumPerZ = 0;

            for(int i=0; i<nofClasses; i++){

                int count = predictionHist[ zi ][i];
                sumPerZ += count;
                System.out.print(count + "  ");
            }
            for(int i=0; i<nofClasses; i++){

                firingProb[zi][i] = predictionHist[ zi ][i]/(double)sumPerZ;
            }

            arrConditionalEntropy[ zi ] = ConditionalEntropy.calculate(firingProb[zi]);
                    
            System.out.print("  \u03A3  " + sumPerZ);
            System.out.print("  condEntr  " + arrConditionalEntropy[ zi ]);
            System.out.println();
            
        }
        System.out.println("No winner: "+noWinnerCount);
        
        String strWeightFile = ".\\data\\output\\weightsMNISTSet.csv";
        saveWeights(strWeightFile, arrZNeurons);
        
    }// testOnMNIST()
    
    public static void saveWeights(String par_strOutputFilename, ZNeuron [] par_arrZNeurons){
        
        int nofLearners = par_arrZNeurons.length;
        int nofNodes = par_arrZNeurons[0].getNofEvidence();
        double [] arrWeights = new double [ nofLearners*nofNodes ];
        int weightArrayIndex = 0;
        for(int zi=0; zi<nofLearners; zi++){
            
            System.arraycopy(par_arrZNeurons[ zi ].getWeights(), 0, arrWeights, weightArrayIndex, nofNodes);
            
            weightArrayIndex += nofNodes;
        }
        
        FileIO.saveArrayToCSV(arrWeights, nofLearners, nofNodes, par_strOutputFilename);
    }   
    
    public static void saveBiases(String par_strOutputFilename, ZNeuron [] par_arrZNeurons){
        
        int nofLearners = par_arrZNeurons.length;
        double [] arrBiases = new double [ nofLearners ];
        for(int zi=0; zi<nofLearners; zi++){
            
            arrBiases[ zi ] = par_arrZNeurons[ zi ].getBias();
        }
        
        FileIO.saveArrayToCSV(arrBiases, nofLearners, 1, par_strOutputFilename);
    }   
    
    public static void saveMasks(String par_strOutputFilename, ActivityMask [] par_arrMasks){
        
        int nofMasks = par_arrMasks.length;
        int nofNodes = par_arrMasks[0].get_nof_nodes();
        double [] arrNodes = new double [ nofMasks*nofNodes ];
        int nodeArrayIndex = 0;
        for(int i=0; i<nofMasks; ++i){
            
            //System.arraycopy(par_arrMasks[ i ].get_mask(), 0, arrNodes, nodeArrayIndex, nofNodes);
            System.arraycopy(par_arrMasks[ i ].get_activity(), 0, arrNodes, nodeArrayIndex, nofNodes);
            nodeArrayIndex += nofNodes;
        }
        
        FileIO.saveArrayToCSV(arrNodes, nofMasks, nofNodes, par_strOutputFilename);
    }
    
    public static void saveResponses(String par_strOutputFilename, int [][] par_arrResponses){
        
        FileIO.saveArrayToCSV(par_arrResponses, par_strOutputFilename);
    }
    
    public static void savePredictionStats(String par_strOutputFilename, double [][] par_firingProbs, double [] par_normCondEntropy){
        
        int nofLearners = par_normCondEntropy.length;
        double [][] arrPredictionSats = new double [ nofLearners ][];
        for(int li=0; li<nofLearners; li++){
            
            int nofCauses = par_firingProbs[li].length;
            arrPredictionSats[li] = new double[ nofCauses+1 ];
            System.arraycopy(par_firingProbs[li], 0, arrPredictionSats[li], 0, nofCauses);
            arrPredictionSats[li][ nofCauses ] = par_normCondEntropy[li];
        }
        FileIO.saveArrayToCSV(arrPredictionSats, par_strOutputFilename);
    }
    
    double [][] normConditionalEntropy(int[][] arrResponse){
        
        double arrNormConditionalEntropy[][];
        
        arrNormConditionalEntropy = null;
                
        return arrNormConditionalEntropy;
        
    }
}
