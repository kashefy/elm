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
import java.util.Arrays;
import model.utils.ModelUtils;

public class PopulationCodeTest {

    
    public void gradientMAXPopCode(){
        
        GradientMAXPopulationCode softMaxPopCoder = new GradientMAXPopulationCode();
        int nofInputs = 4;
        int nfanOut = 1;
        
        double [] input = new double[ nofInputs ];
        ModelUtils.rand(input);
        
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
        
    public void maxPopCode(){
        
        MAXPopulationCode maxPopCoder = new MAXPopulationCode();
        int nofInputs = 3;
        int nfanOut = 1;
        
        double [] input = new double[ nofInputs ];
        ModelUtils.rand(input);
        
        double [] popCode;
        
        maxPopCoder.init(nofInputs, nfanOut);
        popCode = maxPopCoder.calcStateOfNeurons(input);
        
        System.out.println(Arrays.toString(input));
        System.out.println(Arrays.toString(popCode));
    }
    
    public void rankPopulationCode(){
        
        int n = 3;
        double [] input = new double[n];
        ModelUtils.rand(input);
        
        AbstractPopulationCode pop_coder = new RankPopulationCode();
        pop_coder.init(n, n);
        double[] pop_code = pop_coder.calcStateOfNeurons(input);
        
        System.out.println(Arrays.toString(input));
        System.out.println(Arrays.toString(pop_code));
    }
}
