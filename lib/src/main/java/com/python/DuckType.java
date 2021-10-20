import java.util.*;

import errors.*;

public class DuckType {
    public static void test() {
      DuckType duck = new DuckType(new DuckType(-3,-2,-1),0,new DuckType(1,2,3));
      DuckType cat = new DuckType(1);
      System.out.println(duck);
      System.out.println(duck.get(0));
      System.out.println(cat.asRaw(cat.internalType));
      //cat.add(cat);
      //cat.get(1);
    }
    
    ArrayList<DuckType> asList = new ArrayList<>();
    Object asObj;
    final Class<?> internalType;
    
    public <T> DuckType(T... params) {
        if (params.length == 0) {
        	internalType = null;
            return;
        }
        else if (params.length == 1) {
            asObj = params[0];
            internalType = asObj.getClass();
            return;
        }
        
        for (T obj : params)
            asList.add(new DuckType(obj));
        internalType = ArrayList.class;
    }
    
    public DuckType get(int index) {
        if (index > -1 && index < asList.size())
            return asList.get(index);
        else if (asList.size() == 0)
        	throw new AttributeError(String.format("'%s' object is not subscriptable", internalType.getSimpleName()));
        else
            throw new IndexError("list index out of range");
    }
    
    /*TODO: doesn't work
    public DuckType add(DuckType other) {
        if (other.asObj instanceof Number) {
            return new DuckType(((Number)(other.asObj)).doubleValue() + ((Number)(this.asObj)).doubleValue());
        }
        
        return this.asObj.add(other.asObj);
    }*/
    
    public <T> T asRaw(Class<T> type) {
    	return type.cast(asObj);
    }
    
    public String toString() {
        if (asObj != null)
            return asObj.toString();
        
        return asList.toString();
    }
}