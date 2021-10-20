import java.util.*;

// create DuctType extends ParseArguents?
abstract class Py { // abstract = can't create object
	public static void main(String[] args) {  //TODO: easier maps and arrays
		print(null, null);
		
		print("#1", new Object[] {1,2,3,4,5});
		Integer[] bla1 = new Integer[] {1,2,3,4,5};
    	Integer[] bla2 = new Integer[] {6,7,8,9,0,69};
		for(Object bla : zipAll(bla1, bla2))
			print(bla);
		// TODO: could lead to security vulnerability, kwargs being passed in arrays
		print("hello", "world", "end=;\n", "sep= | ");
		String name = input("Name: ");
		name = (name.equals("")) ? "" : ' '+name;
		print(formatStr("{} { time }{ name }!", "good", "time=evening", "name=" + name));
		print("123", "end=YEET", "end=;\n", "sep= | ");
		print(formatStr("{} + {} = {}", 1, 2, 3));
		String bla = input(">>>  ");
		print("testing...", ' ' + bla);
		print("randint:", randint(0, 10));
		print("min:", min(5,1,-4));
		print("char[] contains e:", contains(new Character[] {'a', 'e', 'i', 'o', 'u'}, 'e'));
		print("hello contains ell:", contains("hello", "ell"));
		//print(str(1));
		print(list(range(10)));
	}
	
	public static void print(Object... param) { // TODO: protect against null
		/*
		 * param defaults: "text1", "text2", "sep= ", "end=\n"
		 * sep: string to display between text1 & 2
		 * end: string to display at end of line
		 */
		ParseArguments params = new ParseArguments(allValues(param).toArray());
		int i = 0;
		for (String message : params.args) {
			if (i > 0) {  // prevent separator from appearing at end
				String seperator = params.kwargs.get("sep");
				System.out.print((seperator != null) ? seperator : ' ');
			}
			i++;
			System.out.print(message);
		}
		String EOL = params.kwargs.get("end");  // Nullable
		System.out.print((EOL != null) ? EOL : '\n');
	}
	
	/*static String str(Object object) {
		return object.toString();
	}*/
	
	
	public static String input(String prompt) {
		@SuppressWarnings("resource")
		Scanner reader = new Scanner(System.in);  // Reading from System.in
		System.out.print(prompt);
		String responce = reader.nextLine();
		//reader.close(); don't close or else subsequent calls will fail
		
		return responce;
	}
	
	public static String input() {
		return input("");
	}
	
	public static int randint(int start, int end) {
		int diff = end-start+1;
		return (int)(Math.random()*diff)+start;
	}
	
	/*public static int randint(int end) {
		return randint(0, end);
	}*/
	
	public static double min(double firstVal, double... vals) { // TODO: can't accept arrays or ArrayLists
		double smallest = firstVal;
		for (double val : vals) {
			if (val < smallest)
				smallest = val;
		}
		return smallest;
	}
	
	public static double max(double firstVal, double... vals) {
		double largest = firstVal;
		for (double val : vals) {
			if (val > largest)
				largest = val;
		}
		return largest;
	}
	
	public static boolean contains(Object[] iterable, Object subunit) {
		for (Object item : iterable) {
			if (item.equals(subunit)) return true;
		}
		return false;
	}
	
	public static boolean contains(String iterable, String subunit) {
		return iterable.indexOf(subunit) != -1;
	}
	
	public static Object[] zip(Object[]... allVals) {
		int width = allVals[0].length;
		for (Object[] val : allVals) { // TODO: turn into private function?
			if (val.length < width)
				width = val.length;
		}
		
		int height = allVals.length;
		
		Object[][] result = new Object[width][height];
		for (int col=0; col<width; col++) {
			for (int row=0; row<height; row++) {
				result[col][row] = allVals[row][col];
			}
		}
		return result;
	}
	
	public static Object[] zipAll(Object[]... allVals) {
		int width = allVals[0].length;
		for (Object[] val : allVals) { // TODO: turn into private function?
			if (val.length > width)
				width = val.length;
		}
		
		int height = allVals.length;
		
		Object[][] result = new Object[width][height];
		for (int col=0; col<width; col++) {
			for (int row=0; row<height; row++) {
				try {
					result[col][row] = allVals[row][col];
				} catch(Exception E) {
					result[col][row] = null;
				}
			}
		}
		return result;
	}
	
	public static <T> ArrayList<T> list(Iterable<T> iterable) {
		ArrayList<T> list = new ArrayList<>();
		for (T item: iterable) {
			list.add(item);
		}
		return list;
	}
	
	public static Range range(int start, int stop) {
        return new Range(start, stop);
    }
    
    public static Range range(int start, int stop, int step) {
        return new Range(start, stop, step);
    }
    
    public static Range range(int stop) {
        return new Range(stop);
    }
	
	public static String formatStr(String stringIn, Object... options) {
		/*
		 * Example:
		 * {} {time} {name} -> good, time=evening, name=Ethan
		 * 
		 * Note: will NOT give errors if too many/little options provided
		 */
		String stringOut = stringIn;
		for (Object replaceWithObj : options) {
			String replaceWith = replaceWithObj.toString();
			if (replaceWith.contains("=")) {
				String[] dict = replaceWith.split("=");
				String key = dict[0];
				String value = "";
				if (dict.length > 1)
					value = dict[1];
				stringOut = stringOut.replaceAll("\\{ *" + key + " *\\}", value);
				// match value inside curly braces
			} else {
				stringOut = stringOut.replaceFirst("\\{ *\\}", replaceWith);
				// match any curly braces
			}
		}
		return stringOut;
	}
	
	public static String capitalize(String in) {
		return ("" + in.toCharArray()[0]).toUpperCase() + in.substring(1);
	}
	
	private static <T> ArrayList<T> allValues(T[] nestedArray) {
		if (nestedArray == null) return null;
		
		ArrayList<T> everything = new ArrayList<T>();
		for (T item : nestedArray) {
			if (item != null && item.getClass().isArray()) { // if array contains nested arrays
				for(T value : allValues((T[]) item)) {
					everything.add(value);
					//System.out.println(value);
				}
			} else {
				everything.add(item);
			}
		}
		return everything;
	}
}