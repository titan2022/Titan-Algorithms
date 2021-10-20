import java.util.Iterator;

public class Range implements Iterable<Integer> {

    private int start = 0;
    private int stop;
    private int step = 1;

    public Range(int start, int stop) {
        this.start = start;
        this.stop = stop;
    }
    
    public Range(int start, int stop, int step) {
        this.start = start;
        this.stop = stop;
        this.step = step;
    }
    
    public Range(int stop) {
        this.stop = stop;
    }

    public Iterator<Integer> iterator() {
        Iterator<Integer> it = new Iterator<Integer>() {

            private int currentIndex = start;

            public boolean hasNext() {
                return currentIndex < stop;
            }

            public Integer next() {
            	currentIndex += step;
                return currentIndex - step;
            }

            /*public void remove() {
                throw new UnsupportedOperationException();
            }*/
        };
        return it;
    }
    
    public static void main(String args[]) {
    	for(int i : new Range(1, 5, 2)) {
    		System.out.println(i);
    	}
    }
}