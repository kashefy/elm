/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.utils;

/**
 *
 * @author woodstock
 */
public abstract class AbstractParams {
    
    public abstract void load(String par_path);
    
    public static int resolve_int_double_node(Object par_node, int par_value){
        
        try{
            par_value = (int) par_node;
        }catch(ClassCastException e){
            par_value = (int)(double)par_node;
        }
        return par_value;
    }
    
    public static double resolve_int_double_node(Object par_node, double par_value){
        
        try{
            par_value = (double)par_node;
        }catch(ClassCastException e){
            par_value = (double)(int)par_node;
        }
        return par_value;
    }
}
