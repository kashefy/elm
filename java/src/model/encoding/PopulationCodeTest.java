/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 *
 * @author woodstock
 */
import java.util.Random;

public class PopulationCodeTest {

    
    public void testGradientMAXPopCode(){
        
        GradientMAXPopulationCode softMaxPopCoder = new GradientMAXPopulationCode();
        int nofInputs = 4;
        int nfanOut = 1;
        
        Random generator = new Random();
        double [] input = new double[ nofInputs ];
        
        for(int i=0; i<nofInputs; i++){
            
            input[i] = generator.nextDouble();
        }
        
        double [] popCode;
        
        softMaxPopCoder.init(nofInputs, nfanOut);
        popCode = softMaxPopCoder.calcStateOfNeurons(input);
        
        int nofNodes = softMaxPopCoder.getnofNodes();
        for(int i=0; i<nofInputs; i++){
        
            System.out.print(input[i]+" ");
        } 
        System.out.println();
        for(int i=0; i<nofNodes; i++){
        
            System.out.print(popCode[i]+" ");
        }       
        
        System.out.println(" bi = " + softMaxPopCoder.getBiasIndex());
    }
        
    public void testMAXPopCode(){
        
        MAXPopulationCode maxPopCoder = new MAXPopulationCode();
        int nofInputs = 3;
        int nfanOut = 1;
        
        Random generator = new Random();
        double [] input = new double[ nofInputs ];
        
        for(int i=0; i<nofInputs; i++){
            
            input[i] = generator.nextDouble();
        }
        double [] popCode;
        
        maxPopCoder.init(nofInputs, nfanOut);
        popCode = maxPopCoder.calcStateOfNeurons(input);
        
        int nofNodes = maxPopCoder.getnofNodes();
        for(int i=0; i<nofInputs; i++){
        
            System.out.print(input[i]+" ");
        } 
        System.out.println();
        for(int i=0; i<nofNodes; i++){
        
            System.out.print(popCode[i]+" ");
        }        
    }
}
