package errors;

public class PyError extends RuntimeException {
	private static final long serialVersionUID = -2409247464842024490L;

	public static void main(String[] args) throws PyError, Exception {
		test();
		throw new Exception();
	}
	
	private static void test() throws PyError {
		throw new PyError();
	}
	
	public PyError() {
		this("");
	}

	public PyError(String message) {
		//super("Traceback (most recent call last):\n");
		System.err.println("Traceback (most recent call last):");
		Thread.setDefaultUncaughtExceptionHandler((t, e) -> {
			for (StackTraceElement step : e.getStackTrace())
				System.err.println(String.format("    File \"%s\", line %d, in %s",
						step.getFileName(),
						step.getLineNumber(),
						step.getMethodName() ));
			System.err.println(this.getClass().getName() + ": " + message);
		});
	}
}
