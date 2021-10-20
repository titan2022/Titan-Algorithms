import java.util.*;

public class ParseArguments {
	final Map<String, String> kwargs = new HashMap<String, String>();  // TODO: make private?
	final ArrayList<String> args = new ArrayList<String>();
	
	public ParseArguments(Object... params) {
		for (Object arg : params) {
			if (arg == null) {
				addArgs("null");
				continue;
			}
			
			String argStr = arg.toString();
		    if (argStr.contains("=") && !argStr.split("=")[0].contains(" ")) {  // is named argument
		    	// also checks that key does not contain any spaces
		    	addKwargs(argStr);
		    } else {  // is unnamed argument
		    	if (argStr.toCharArray()[0] == ' ')
		    		argStr = argStr.substring(1);
		    	// if first char space, remove it
		    	addArgs(argStr);
		    }
		}
	}
	
	public ParseArguments addArgs(Object... params) {
	    for (Object arg : params) {
	        args.add(arg.toString());
	    }
	    return this;
	}
	
	public ParseArguments addKwargs(String... params) {
	    for (String item : params) {
	        String[] kwarg = item.split("=");
	        String key = kwarg[0];
		    String value = kwarg[1];
		    kwargs.put(key, value);
	    }
	    return this;
	}
}